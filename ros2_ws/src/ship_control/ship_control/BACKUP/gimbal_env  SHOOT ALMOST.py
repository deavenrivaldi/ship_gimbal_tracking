"""
GIMBAL CONTROLLER NODE
Package    : ship_control
"""

import os
import math
import time
import random
import numpy as np
import subprocess

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityWrench, Entity
from ship_msgs.srv import Fire
from ros_gz_interfaces.srv import DeleteEntity

import gymnasium as gym
from gymnasium import spaces

from ros_gz_interfaces.msg import Contacts  

import tf2_ros
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory



class ShipGimbalEnv(gym.Env, Node):
    JOINT_NAMES = ['yaw_joint', 'pitch_joint', 'roll_joint']
    JOINT_LIMITS = {
        'yaw_joint':   (-1000, 1000), # 允許 360 度無限旋轉
        'pitch_joint': (-1.57, 1.57),
        'roll_joint':  (-1.57, 1.57),
    }
    TIME_FROM_START = 0.05 
    VISION_TIMEOUT = 0.5

    def __init__(self):
        Node.__init__(self, 'gimbal_rl_env')
        gym.Env.__init__(self)

        self.declare_parameter('world_name', 'gimbal_world')
        self.world_name = self.get_parameter('world_name').value
        
        self.search_anchor_yaw = 0.0   
        self.search_anchor_pitch = 0.0 
        self.pan_cmd  = 0.0
        self.tilt_cmd = 0.0
        self.roll_correction  = 0.0
        self.pitch_correction = 0.0
        self.last_action_yaw = 0.0
        self.last_action_pitch = 0.0
        self.current_positions = {name: 0.0 for name in self.JOINT_NAMES}
        
        self.last_vision_time = 0.0 

        self.sub_vision = self.create_subscription(Vector3, '/gimbal/angle_command', self.vision_callback, 10)
        self.sub_imu = self.create_subscription(Vector3, '/gimbal/roll_correction', self.imu_correction_callback, 10)
        self.sub_joints = self.create_subscription(JointState, f'/world/{self.world_name}/model/gimbal/joint_state', self.joint_state_callback, 10)
        self.pub_trajectory = self.create_publisher(JointTrajectory, '/gimbal/joint_trajectory', 10)

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(14,), dtype=np.float32)
        
        self.last_pan_err_obs = 0.0
        self.last_tilt_err_obs = 0.0

        self.last_seen_pan = 0.0
        self.last_seen_tilt = 0.0
        self.step_count = 0
        self.lock_on_counter = 0  # 🌟 新增：重置雷達鎖定計時器
        self.max_steps = 1000
        self.smoothed_roll = 0.0

        self.episode_count = 0
        
        self.last_step_time = time.time()
        self.was_lost = True 
        self.thrust_timer = 0
        self.last_shoot_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.fire_client = self.create_client(Fire, '/fire')

        self.physical_hit = False
        self.last_registered_hit_time = 0.0 # 痛覺冷卻鎖
        
        self.sub_hit = self.create_subscription(
            Contacts, 
            '/world/gimbal_world/model/wamv/link/person_link/sensor/person_contact/contact', 
            self.hit_callback, 
            10
        )

        self.has_seen_target = False
        
        # ==========================================
        # 🎮 課程學習 (Curriculum Learning) 狀態變數
        # ==========================================
        self.current_stage = 1         # 關卡進度 (現在擴增至 5 關)
        self.stage_hit_count = 0       
        self.target_on_left = True     
        
        self.get_logger().info('✅ Gimbal RL Environment Ready! 5-Stage Curriculum Loaded.')
    
    def get_camera_world_pose(self):
        try:
            return self.tf_buffer.lookup_transform(
                'gimbal_world',
                'gimbal_wam_v/gimbal/camera_link',
                rclpy.time.Time(nanoseconds=0)
            )
        except Exception as e:
            self.get_logger().warning(f'TF2 lookup failed: {e}', throttle_duration_sec=1.0)
            return None

    def quaternion_to_direction(self, q) -> Vector3:
        direction = Vector3()
        direction.x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        direction.y = 2.0 * (q.x * q.y + q.w * q.z)
        direction.z = 2.0 * (q.x * q.z - q.w * q.y)
        return direction

    def shoot(self, force=1500.0):
        transform = self.get_camera_world_pose()
        if transform is None:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        direction = self.quaternion_to_direction(q)
        request = Fire.Request()
        
        request.position = Point(
            x=t.x + direction.x * 1.0,
            y=t.y + direction.y * 1.0,
            z=t.z + direction.z * 1.0
        )
        request.direction = direction
        request.force = force
        self.fire_client.call_async(request)

    def vision_callback(self, msg):
        self.pan_cmd = math.radians(msg.x)
        self.tilt_cmd = math.radians(msg.y)
        self.last_vision_time = time.time()
        self.has_seen_target = True

    def imu_correction_callback(self, msg):
        self.roll_correction  = math.radians(msg.x)
        self.pitch_correction = math.radians(msg.y)

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = position

    def build_trajectory(self, positions, duration_sec):
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [positions[name] for name in self.JOINT_NAMES]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec - int(duration_sec)) * 1e9))
        traj.points = [point]
        return traj

    def _get_obs(self):
        time_since_vision = time.time() - self.last_vision_time
        is_lost = 1.0 if (time_since_vision > 2.0 or not self.has_seen_target) else 0.0
        normalized_yaw = math.atan2(math.sin(self.current_positions['yaw_joint']), math.cos(self.current_positions['yaw_joint']))
        
        yaw_diff_rad = self.current_positions['yaw_joint'] - self.last_seen_pan
        shortest_yaw_diff = math.atan2(math.sin(yaw_diff_rad), math.cos(yaw_diff_rad))
        tilt_diff = self.current_positions['pitch_joint'] - self.last_seen_tilt
        
        current_pan_err = self.pan_cmd if not is_lost else 0.0
        current_tilt_err = self.tilt_cmd if not is_lost else 0.0
        
        pan_velocity = (current_pan_err - self.last_pan_err_obs) / self.TIME_FROM_START
        tilt_velocity = (current_tilt_err - self.last_tilt_err_obs) / self.TIME_FROM_START
        
        self.last_pan_err_obs = current_pan_err
        self.last_tilt_err_obs = current_tilt_err
        
        # ========================================================
        # 🌟 核心修復：把無限膨脹的 last_seen_pan 強制折疊回 -pi 到 pi
        # ========================================================
        normalized_last_seen_pan = math.atan2(math.sin(self.last_seen_pan), math.cos(self.last_seen_pan))

        return np.array([
            self.pan_cmd, self.tilt_cmd, 
            self.roll_correction, self.pitch_correction,
            normalized_yaw,  
            self.current_positions['pitch_joint'], self.current_positions['roll_joint'],
            is_lost,
            normalized_last_seen_pan,  # 👈 兇手在這裡！把原本的 self.last_seen_pan 換成這變數！
            time_since_vision * 0.1,
            shortest_yaw_diff, 
            tilt_diff,
            pan_velocity, 
            tilt_velocity
        ], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0

        try:
            time.sleep(0.1)
            self.episode_count += 1
            
            # 主船強制歸位
            reset_main_ship_cmd = (
                f"gz service -s /world/{self.world_name}/set_pose "
                f"--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--req 'name: \"gimbal_wam_v\", position: {{x: 8.0, y: 0.0, z: 0.2}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'"
            )
            subprocess.run(reset_main_ship_cmd, shell=True, capture_output=True, text=True)
            time.sleep(0.02)

            # ==========================================
            # 🎮 課程學習：五大關卡空投與物理動態設定
            # ==========================================
            base_t = 0.0
            diff_t = 0.0
            
            if self.current_stage == 1:
                rand_x, rand_y = 12.0, 6.0 
                qz, qw = 0.0, 1.0
                self.get_logger().info(f"🎓 [STAGE 1] 尋敵特訓 ({self.stage_hit_count}/10) | 目標空投於正後方靜止。")
                
            elif self.current_stage == 2:
                rand_x = 18.0 
                if self.target_on_left:
                    rand_y = 5.0
                    self.get_logger().info(f"🎓 [STAGE 2] 射擊特訓 ({self.stage_hit_count}/10) | 目標【左前方】靜止。")
                else:
                    rand_y = -5.0
                    self.get_logger().info(f"🎓 [STAGE 2] 射擊特訓 ({self.stage_hit_count}/10) | 目標【右前方】靜止。")
                qz, qw = 0.0, 1.0
                
            elif self.current_stage == 3:
                # 🌀 第三關：前方繞小圈圈
                rand_x, rand_y = 18.0, 0.0
                rand_yaw = 1.57 # 面向左邊起步
                qz, qw = math.sin(rand_yaw / 2.0), math.cos(rand_yaw / 2.0)
                base_t = 150.0
                diff_t = 150.0  # 左引擎 300, 右引擎 0 -> 極度右滿舵，原地繞小圈
                self.get_logger().info(f"🌀 [STAGE 3] 動態追蹤 ({self.stage_hit_count}/10) | 目標於前方繞小圈圈！")
                
            elif self.current_stage == 4:
                # 🪐 第四關：繞著主船公轉
                rand_x, rand_y = 18.0, 0.0
                rand_yaw = 1.57 # 面向左邊起步
                qz, qw = math.sin(rand_yaw / 2.0), math.cos(rand_yaw / 2.0)
                base_t = 300.0
                diff_t = 60.0  # 左引擎 360, 右引擎 240 -> 大半徑右轉，完美環繞主船公轉
                self.get_logger().info(f"🪐 [STAGE 4] 環繞追蹤 ({self.stage_hit_count}/10) | 目標正圍繞本船 360 度公轉！")

            else:
                # 🚀 第五關：全域隨機海象亂鬥 (原本的第三關)
                center_x, center_y = 8.0, 0.0  
                r = random.uniform(8.0, 15.0)
                theta = random.uniform(-math.pi, math.pi)
                rand_x = center_x + r * math.cos(theta)
                rand_y = center_y + r * math.sin(theta)
                rand_yaw = random.uniform(-math.pi, math.pi) 
                qz, qw = math.sin(rand_yaw / 2.0), math.cos(rand_yaw / 2.0)
                
                base_t = random.uniform(-2, 4) * 100  
                diff_t = random.uniform(-1.5, 1.5) * 100  
                self.get_logger().info(f"🔥 [STAGE 5] 終極實戰 | 目標隨機空投於 ({rand_x:.1f}, {rand_y:.1f})")

            # 空投目標船
            set_pose_cmd = (
                f"gz service -s /world/{self.world_name}/set_pose "
                f"--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--req 'name: \"wamv\", position: {{x: {rand_x}, y: {rand_y}, z: 0.2}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'"
            )
            subprocess.run(set_pose_cmd, shell=True, capture_output=True, text=True)

            # 執行初始動力設定 (這會決定它是繞圈還是亂跑)
            time.sleep(0.02)
            thrust_cmd = f"gz topic -t /model/wamv/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t + diff_t}\" & " \
                         f"gz topic -t /model/wamv/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t - diff_t}\""
            subprocess.run(thrust_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        except Exception as e:
            self.get_logger().error(f"重置失敗: {e}")
            
        self.pan_cmd = 0.0
        self.tilt_cmd = 0.0
        self.last_seen_pan = 0.0  
        self.last_seen_tilt = 0.0   
        self.has_seen_target = False  
        self.last_pan_err_obs = 0.0
        self.last_tilt_err_obs = 0.0


        # ==========================================
        # 🧹 1. 大腦與記憶強制洗腦 (軟體歸零)
        # ==========================================
        self.last_action_yaw = 0.0
        self.last_action_pitch = 0.0
        self.lock_on_counter = 0

        # ==========================================
        # 🛑 2. 物理馬達緊急煞車 (原地凍結，不強制回歸)
        # ==========================================
        # 先快速更新一次 ROS 2 狀態，確保等一下拿到的實體位置是最新的
        rclpy.spin_once(self, timeout_sec=0.02)
        
        # 取得當下真實的物理位置
        current_yaw = self.current_positions['yaw_joint']
        current_pitch = self.current_positions['pitch_joint']

        # 🌟 核心修復：把大腦的虛擬指令直接對齊現在的實體位置
        self.cmd_yaw = current_yaw
        self.cmd_pitch = current_pitch

        # 打造一個「原地凍結」的指令，取代原本的「絕對零度」
        freeze_positions = {
            'yaw_joint': current_yaw,
            'pitch_joint': current_pitch,
            'roll_joint': self.current_positions['roll_joint']
        }

        # 🌟 煞車迴圈：連續發送凍結指令，強迫蓋掉 ROS 2 佇列裡的舊指令，讓它死死停在原地
        for _ in range(5):
            self.pub_trajectory.publish(self.build_trajectory(freeze_positions, 0.05))
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # 因為不強制回正，只需要給予極短的緩衝時間確認煞車即可
        time.sleep(0.1)
        self.last_vision_time = time.time()  
        self.was_lost = True

        # ==========================================
        # 🧽 3. 抽乾 ROS 2 舊視角影像與感測器垃圾
        # ==========================================
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)

        # ==========================================
        # 🌟 4. 核心修復：強制抹除前世的殘留痛覺！
        # ==========================================
        self.physical_hit = False
        self.last_registered_hit_time = time.time()

        return self._get_obs(), {}

    def hit_callback(self, msg):
        current_time = time.time()
        # 🌟 痛覺冷卻鎖：0.5 秒內只算一次命中
        if current_time - self.last_registered_hit_time < 0.5:
            return

        for contact in msg.contacts:
            col1 = contact.collision1.name
            col2 = contact.collision2.name
            
            if 'projectile' in col1 or 'projectile' in col2:
                self.physical_hit = True
                self.last_registered_hit_time = current_time
                break

    def step(self, action):
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.002)

        current_time = time.time()
        time_gap = current_time - self.last_step_time
        
        if time_gap > 0.3:
            self.last_vision_time += time_gap  
            self.last_step_time = current_time

        self.step_count += 1

        # ==========================================
        # 🌊 只有進入第五關，海浪才會不規律改變推力
        # 第一到第四關，推力都會維持在 reset 時設定的「完美繞圈」數值
        # ==========================================
        self.thrust_timer += 1
        if self.thrust_timer >= 60:
            self.thrust_timer = 0
            if self.current_stage == 5:
                base_t = random.uniform(-3, 5) * 100  
                diff_t = random.uniform(-2.5, 2.5) * 100  
                thrust_cmd = f"gz topic -t /model/wamv/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t + diff_t}\" & " \
                             f"gz topic -t /model/wamv/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t - diff_t}\""
                subprocess.Popen(thrust_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        action_yaw = float(action[0])
        action_pitch = float(action[1])
        action_roll = float(action[2])
        action_shoot_trigger = float(action[3]) 

        # 🌟 加上 max(0.0, ...) 防止時間變成負數！
        time_since_vision = max(0.0, time.time() - self.last_vision_time) 
        is_lost = (time_since_vision > self.VISION_TIMEOUT) or (not self.has_seen_target)
        
        pan_err = self.pan_cmd if not is_lost else 0.0
        tilt_err = self.tilt_cmd if not is_lost else 0.0
        tracking_error = math.sqrt(pan_err**2 + tilt_err**2)
        tracking_error_deg = math.degrees(tracking_error)
        
        max_yaw_step = 0.10   
        max_pitch_step = 0.05 
        #max_roll_step = 0.10

        # ========================================================
        # 🌟 核心修復：使用 cmd_yaw 進行平滑累積，拒絕物理落後造成的指令斷層
        # ========================================================
        self.cmd_yaw = self.cmd_yaw - (action_yaw * max_yaw_step)
        self.cmd_pitch = self.cmd_pitch + (action_pitch * max_pitch_step)
        yaw_target = self.cmd_yaw      # 👈 你漏掉了這行！
        pitch_target = self.cmd_pitch  # 👈 你漏掉了這行！

        # ========================================================
        # 🔒 安全鎖重啟：沒收 AI 的 Roll 控制權，交由 IMU 絕對物理防抖！
        # ========================================================
        roll_target = -self.roll_correction

        action_yaw_diff = action_yaw - self.last_action_yaw
        action_pitch_diff = action_pitch - self.last_action_pitch
        
        self.last_action_yaw = action_yaw
        self.last_action_pitch = action_pitch

        yaw_target = np.clip(yaw_target, self.JOINT_LIMITS['yaw_joint'][0], self.JOINT_LIMITS['yaw_joint'][1])
        pitch_target = np.clip(pitch_target, self.JOINT_LIMITS['pitch_joint'][0], self.JOINT_LIMITS['pitch_joint'][1])
        roll_target = np.clip(roll_target, self.JOINT_LIMITS['roll_joint'][0], self.JOINT_LIMITS['roll_joint'][1])

        self.current_positions['yaw_joint'] = yaw_target
        self.current_positions['pitch_joint'] = pitch_target
        self.current_positions['roll_joint'] = roll_target

        positions = {'yaw_joint': yaw_target, 'pitch_joint': pitch_target, 'roll_joint': roll_target}
        self.pub_trajectory.publish(self.build_trajectory(positions, self.TIME_FROM_START))

        yaw_diff_rad = self.current_positions['yaw_joint'] - self.last_seen_pan
        shortest_yaw_diff_rad = math.atan2(math.sin(yaw_diff_rad), math.cos(yaw_diff_rad))
        last_seen_pan_diff_deg = abs(math.degrees(shortest_yaw_diff_rad))
        last_seen_tilt_diff_deg = abs(math.degrees(self.current_positions['pitch_joint']) - math.degrees(self.last_seen_tilt))
        last_seen_pose_diff_deg = math.sqrt(last_seen_pan_diff_deg**2 + last_seen_tilt_diff_deg**2)

        reward = 0.0
        terminated = False
        
        # ==========================================
        # 🏆 1. 終極目標：命中結算 (提早通關紅利)
        # ==========================================
        if self.physical_hit:
            # 🌟 獎勵縮放：將每步紅利從 10.0 調降為 0.5
            # 這樣最高紅利會從 10000 分降為 5000 分！
            time_bonus = max(0.0, (5000 - self.step_count) * 0.5)
            
            # 總得分 = 基礎 500 + 最高紅利 500 = 最高約 1000 分 (非常健康的數值)
            total_hit_reward = 500.0 + time_bonus
            reward += total_hit_reward
            
            self.get_logger().info(f"💥💥 命中！獲基礎 500 + 紅利 {time_bonus:.0f} = {total_hit_reward:.0f} 分！")
            self.physical_hit = False
            
            # 關卡晉級判定
            if self.current_stage == 1:
                self.stage_hit_count += 1
                if self.stage_hit_count >= 20:
                    self.current_stage = 2
                    self.stage_hit_count = 0
                    self.get_logger().info("🏆🏆🏆 晉級第二關！開始訓練【左右切換追蹤】 🏆🏆🏆")
                terminated = True  
            elif self.current_stage == 2:
                self.stage_hit_count += 1
                if self.stage_hit_count >= 20:
                    self.current_stage = 3
                    self.stage_hit_count = 0
                    self.get_logger().info("🌀🌀🌀 晉級第三關！開始訓練【前方小圈動態追蹤】 🌀🌀🌀")
                else:
                    self.target_on_left = not self.target_on_left  
                terminated = True  
            elif self.current_stage == 3:
                self.stage_hit_count += 1
                if self.stage_hit_count >= 20:
                    self.current_stage = 4
                    self.stage_hit_count = 0
                    self.get_logger().info("🪐🪐🪐 晉級第四關！開始訓練【360度環繞公轉追蹤】 🪐🪐🪐")
                terminated = True
            elif self.current_stage == 4:
                self.stage_hit_count += 1
                if self.stage_hit_count >= 20:
                    self.current_stage = 5
                    self.stage_hit_count = 0
                    self.get_logger().info("🚀🚀🚀 進入最終地獄關卡！海浪隨機動態大亂鬥 🚀🚀🚀")
                terminated = True
            elif self.current_stage == 5:
                terminated = True

        # ==========================================
        # 🎮 2. 日常行為給分 (追蹤、開火、耗能)
        # ==========================================
        ai_wants_to_shoot = (action_shoot_trigger > 0.0)
        can_shoot = (current_time - self.last_shoot_time > 1.0)
        action_magnitude = (action_yaw**2 + action_pitch**2 + action_roll**2)

        if not is_lost:
            # 🟢 生存底薪與水平儀保護
            reward += 1.0 
            roll_error = abs(self.current_positions['roll_joint'] - (-self.roll_correction))
            if roll_error < 0.1:
                reward += 1.0

            # ==========================================
            # 🏆 1 & 2. 階梯式狀態機 + ABS 煞車 + 雷達鎖定
            # ==========================================
            if tracking_error_deg < 2.0:
                self.lock_on_counter += 1
                # 🌟 絕對紅心：唯一的暴富區
                reward += 60.0  
                status_str = "🎯 完美爆頭！(誤差 < 2°)"
                reward -= action_magnitude * 5.0 
                reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 1.0
                
            elif tracking_error_deg < 10.0:
                self.lock_on_counter += 1
                # 🌟 斷層 1：底薪大幅砍到 20，讓它眼紅 60 分
                reward += 50.0  
                status_str = "🟢 穩定鎖定 (誤差 < 10°)"

                raw_gravity = (tracking_error_deg / 2.0) ** 2
                gravity_penalty = min(15.0, raw_gravity) # 上限調低一點，避免跟外圍重疊
                reward -= gravity_penalty

                reward -= action_magnitude * 1.0  
                reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 1.0
                
            elif tracking_error_deg < 20.0:
                self.lock_on_counter = 0  
                # 🌟 斷層 2：底薪降到 5，幾乎被重力井抵銷，這裡是不賺不賠的掙扎區
                reward += 30.0   
                status_str = "🟡 邊緣捕捉 (誤差 < 20°)"

                raw_gravity = (tracking_error_deg / 4.0) ** 2
                gravity_penalty = min(30.0, raw_gravity)
                reward -= gravity_penalty

                reward -= action_magnitude * 0.5  
                reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 1.0
                
            else:
                self.lock_on_counter = 0  
                status_str = "🟠 視野邊緣危險區"
                # 🌟 斷層 3：拔除底薪！外圍就是純粹的痛苦，逼它趕快轉頭
                reward += 30.0 
                
                raw_gravity = (tracking_error_deg / 4.0) ** 2
                gravity_penalty = min(30.0, raw_gravity)
                reward -= gravity_penalty
                
                reward -= action_magnitude * 0.5
                reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 1.0

            # ==========================================
            # 🔫 3. 終極扳機紀律：加入「武器預熱」限制
            # ==========================================
            if ai_wants_to_shoot and can_shoot:
                if self.current_stage == 1:
                    legal_aim_angle = 10.0       
                    blind_fire_penalty = 30.0   
                    required_lock_steps = 5     # 第一關：只需鎖定 0.25 秒
                else:
                    legal_aim_angle = 10.0       
                    blind_fire_penalty = 150.0  
                    required_lock_steps = 5    # 第二關起：必須穩定鎖定 0.25 秒！

                if tracking_error_deg < legal_aim_angle:
                    # 🌟 檢查武器是否完成鎖定？
                    if self.lock_on_counter >= required_lock_steps:
                        # 【合法且穩定的開火】
                        aim_cost = tracking_error_deg * 2.0
                        reward -= aim_cost 
                        
                        self.shoot(force=15000.0)
                        self.last_shoot_time = current_time
                        
                        status_str = f"🔥 精準開火！(偏差 {tracking_error_deg:.1f}°)"
                        self.get_logger().info(f"🎯 [完美擊發] 穩定鎖定達標，誤差 {tracking_error_deg:.1f}°，成功發射！")
                    else:
                        # 【甩狙懲罰】對準了但沒有停下來！
                        reward -= 20.0 
                        self.last_shoot_time = current_time # 照樣卡彈
                        status_str = "🚫 拒絕開火：雷達未完成鎖定！"
                        self.get_logger().warn(f"🚫 [甩狙警告] 企圖掃射！請穩定停留在目標上 0.5 秒再開火！扣 20 分！")
                
                else:
                    # 【非法盲射】
                    reward -= blind_fire_penalty
                    self.last_shoot_time = current_time
                    status_str = f"🚫 盲射鎖死！扣 {blind_fire_penalty} 分！"
                    self.get_logger().warn(f"🚫 [盲射重罰] 誤差 {tracking_error_deg:.1f}°，未達標準！扣 {blind_fire_penalty} 分！")
            
            elif tracking_error_deg < 2.0 and not ai_wants_to_shoot:
                if self.lock_on_counter >= 10:
                    status_str = "🎯 鎖定完畢！(武器已就緒，可隨時開火)"
                else:
                    status_str = f"⏳ 鎖定中... ({self.lock_on_counter}/10)"

            self.was_lost = False
            self.last_seen_pan = self.current_positions['yaw_joint'] - self.pan_cmd
            self.last_seen_tilt = self.current_positions['pitch_joint'] + self.tilt_cmd
            
        else:
            # ==========================================
            # 🧭 3. 丟失目標 (全域探索與約束 - Yaw 限定版)
            # ==========================================
            self.was_lost = True
            status_str = "🧭 丟失目標！全域探索中..."

            # 🌟 時間恐慌懲罰不變 (這是全域的急迫感)
            panic_penalty = max(10.0, 10.0 + (time_since_vision * 5.0))
            reward -= panic_penalty

            # 🔴 [懲罰] Pitch 俯仰角安全邊界 (只防止看天看地，不逼迫它移動)
            pitch_pos = self.current_positions['pitch_joint']
            if abs(pitch_pos) > 0.6:  
                pitch_penalty = ((abs(pitch_pos) - 0.6) * 10.0) ** 2
                reward -= pitch_penalty
                status_str = "⚠️ 仰角過大！重擊扣血中..."

            # 🔴 [懲罰] 怠惰與防抖 (🌟 核心修改：徹底放過 Pitch，只抓 Yaw)
            #if abs(action_yaw) < 0.5:
            #    reward -= 15.0  # 只有 Yaw 不認真轉才扣分

            # 這會逼它用「剛剛好能找到人」的速度去轉，而不是死壓著搖桿當陀螺
            reward -= (action_yaw**2) * 2.0
                
            # 防抖也只抓 Yaw！允許 Pitch 在尋敵時保持不動，避免被扣冤枉分
            reward -= (action_yaw_diff**2) * 5.0    
                
        # ==========================================
        # 📊 4. 狀態輸出與強制結束機制
        # ==========================================
        if self.step_count % 5 == 0:
            self.get_logger().info(
                f"[{status_str}] Err: {tracking_error_deg:.2f}° | Reward: {reward:.2f} | Act: [{action_yaw:.2f}, {action_pitch:.2f}]"
            )

        truncated = False
        if self.step_count >= 5000:
            truncated = True
            self.get_logger().info("⏳ 追蹤時間耗盡 (已達 1000 步)，強制重新佈局！")

        timeout_seconds = 10.0 
        if time_since_vision > timeout_seconds:
            reward -= 100.0  
            terminated = True
            self.get_logger().info(f"💀 追丟超時 ({timeout_seconds:.2f} 秒)！獲得懲罰 -100，強制重置世界！")

        obs = self._get_obs()
        
        target_time = 0.005  
        step_execution_time = time.time() - current_time
        if step_execution_time < target_time:
            time.sleep(target_time - step_execution_time)
        
        self.last_step_time = time.time()
        
        return obs, reward, terminated, truncated, {}