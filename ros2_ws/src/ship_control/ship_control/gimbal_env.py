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
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32)
        
        self.last_pan_err_obs = 0.0
        self.last_tilt_err_obs = 0.0

        self.last_seen_pan = 0.0
        self.last_seen_tilt = 0.0

        self.last_seen_yaw_vel = 0.0  # 🌟 新增：目標的水平角速度記憶

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
        self.search_spin_dir = 1.0  # 🌟 新增：預設的死轉方向
        
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
            normalized_last_seen_pan, # 👈 兇手在這裡！把原本的 self.last_seen_pan 換成這變數！ 
            time_since_vision * 0.1,
            shortest_yaw_diff, 
            tilt_diff,
            pan_velocity, 
            tilt_velocity,
            self.last_action_yaw,   # 🌟 救命關鍵：告訴 AI 它的肌肉剛剛在哪！
            self.last_action_pitch,  # 🌟 救命關鍵 2
            # 🌟 換成讀取全域的速度變數
            self.current_yaw_vel,   
            self.current_pitch_vel
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
                noise = random.uniform(-0.5, 0.5) 
                rand_x, rand_y = 10.0+ noise, 6.0 + noise
                qz, qw = 0.0, 1.0
                self.get_logger().info(f"🎓 [STAGE 1] 尋敵特訓 ({self.stage_hit_count}/20) | 目標空投於左前方鏡頭外靜止。")
                
            elif self.current_stage == 2:
                noise = random.uniform(-0.5, 0.5) 
                rand_x = 18.0 + noise
                if self.target_on_left:
                    rand_y = 5.0+ noise
                    self.get_logger().info(f"🎓 [STAGE 2] 射擊特訓 ({self.stage_hit_count}/20) | 目標【左前方】靜止。")
                else:
                    rand_y = -5.0+ noise
                    self.get_logger().info(f"🎓 [STAGE 2] 射擊特訓 ({self.stage_hit_count}/20) | 目標【右前方】靜止。")
                qz, qw = 0.0, 1.0
                
            elif self.current_stage == 3:
                # 🌀 第三關：前方繞小圈圈
                noise = random.uniform(-0.5, 0.5) 
                rand_x, rand_y = -2.0+ noise, 0.0+ noise
                rand_yaw = 1.57 # 面向左邊起步
                qz, qw = math.sin(rand_yaw / 2.0), math.cos(rand_yaw / 2.0)
                base_t = 100.0
                diff_t = 100.0  # 左引擎 300, 右引擎 0 -> 極度右滿舵，原地繞小圈
                self.get_logger().info(f"🌀 [STAGE 3] 動態追蹤 ({self.stage_hit_count}/20) | 目標於前方繞小圈圈！")
                
            elif self.current_stage == 4:
                # 🪐 第四關：繞著主船公轉
                noise = random.uniform(-0.5, 0.5)
                rand_x, rand_y = -2.0+ noise, 0.0+ noise
                rand_yaw = 1.57 # 面向左邊起步
                qz, qw = math.sin(rand_yaw / 2.0), math.cos(rand_yaw / 2.0)
                base_t = 150.0
                diff_t = 75.0  # 左引擎 360, 右引擎 240 -> 大半徑右轉，完美環繞主船公轉
                self.get_logger().info(f"🪐 [STAGE 4] 環繞追蹤 ({self.stage_hit_count}/20) | 目標正圍繞本船 360 度公轉！")

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
        self.has_seen_target = False  
        # 🌟 新增：開局隨機決定雷達死轉的方向 (1.0 = 左滿舵, -1.0 = 右滿舵)
        self.search_spin_dir = random.choice([1.0, -1.0])
        
        # 🌟 新增：標記系統目前是否在盲掃
        self.just_found_target = False

        self.last_pan_err_obs = 0.0
        self.last_tilt_err_obs = 0.0

        # ==========================================
        # 📸 1. 開場快門：先拍下上一局最後一刻的角度 (確保拿到最新物理狀態)
        # ==========================================
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.01)
            
        saved_yaw = self.current_positions['yaw_joint']
        saved_pitch = self.current_positions['pitch_joint']

        # ==========================================
        # 🧹 2. 大腦洗腦，但完全保留物理角度
        # ==========================================
        # 把 AI 的虛擬搖桿指令，直接對齊現在的物理位置
        self.cmd_yaw = saved_yaw
        self.cmd_pitch = saved_pitch
        self.last_action_yaw = 0.0
        self.last_action_pitch = 0.0
        self.lock_on_counter = 0

        # 🌟 關鍵：將「最後看到目標的位置」設為停下的位置，避免開局神經網絡錯亂
        self.last_seen_pan = saved_yaw      
        self.last_seen_tilt = saved_pitch   

        self.last_seen_yaw_vel = 0.0  # 🌟 開局時速度記憶歸零
        
        # ==========================================
        # 🛑 3. 實施電子手煞車 (鎖死在 saved_yaw)
        # ==========================================
        freeze_positions = {
            'yaw_joint': saved_yaw,
            'pitch_joint': saved_pitch,
            'roll_joint': self.current_positions['roll_joint']
        }
        
        # 煞車迴圈：連續發送凍結指令，強迫蓋掉 ROS 2 佇列裡的舊指令，把速度降為 0
        for _ in range(5):
            self.pub_trajectory.publish(self.build_trajectory(freeze_positions, 0.05))
            rclpy.spin_once(self, timeout_sec=0.02)

        # ==========================================
        # 🌟 4. 痛覺神經抹除與狀態重置
        # ==========================================
        self.physical_hit = False
        self.last_registered_hit_time = time.time()
        self.last_vision_time = time.time()  
        self.was_lost = True

        # ==========================================
        # 🏎️ 開局時速表與角度記憶歸零
        # ==========================================
        self.current_yaw_vel = 0.0
        self.current_pitch_vel = 0.0
        self.last_yaw_pos = self.current_positions['yaw_joint']
        self.last_pitch_pos = self.current_positions['pitch_joint']

        return self._get_obs(), {}  # 這是原本的最後一行
        

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
        
        # ==========================================
        # 🏎️ 差分法：用現有角度資料，自己算出雲台的實體角速度
        # ==========================================
        # 1. 取得當前角度
        current_yaw_pos = self.current_positions['yaw_joint']
        current_pitch_pos = self.current_positions['pitch_joint']

        # 2. 為了避免第一步沒有「上一次的角度」而報錯，我們用 getattr 給個預設值
        last_yaw_pos = getattr(self, 'last_yaw_pos', current_yaw_pos)
        last_pitch_pos = getattr(self, 'last_pitch_pos', current_pitch_pos)

        # 3. 確保時間差不能為 0 (避免除以零崩潰)
        dt = max(time_gap, 0.001)

        # 4. 計算角度差 (Yaw 必須處理 360 度折疊問題！)
        yaw_diff_pos = current_yaw_pos - last_yaw_pos
        shortest_yaw_diff_pos = math.atan2(math.sin(yaw_diff_pos), math.cos(yaw_diff_pos))
        pitch_diff_pos = current_pitch_pos - last_pitch_pos

        # 5. 算出我們自己的實體時速！(加上 self. 讓全域都能讀取)
        self.current_yaw_vel = shortest_yaw_diff_pos / dt
        self.current_pitch_vel = pitch_diff_pos / dt

        # 6. 更新記憶，留給下一步用
        self.last_yaw_pos = current_yaw_pos
        self.last_pitch_pos = current_pitch_pos

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

        # ========================================================
        # 📥 1. 取得 AI 大腦輸出的「原始狂野指令」
        # ========================================================
        raw_action_yaw = float(action[0])
        raw_action_pitch = float(action[1])
        action_roll = float(action[2])
        action_shoot_trigger = float(action[3]) 

        # ========================================================
        # 🤖 核心升級：混合式控制 (FSM 系統接管盲掃)
        # 如果還沒看到目標，直接沒收 AI 控制權，由系統強制死轉！
        # ========================================================
        if not self.has_seen_target:
            raw_action_yaw = self.search_spin_dir*0.2  # 強制全速死轉
            raw_action_pitch = 0.0                 # 強制視線貼齊海平面
            self.cmd_pitch = 0.0                   # 🌟 暴力破解：無視慣性，直接把實體雲台目標角度壓回地平線 (0度)！
            # 🌟 修復 2：接上煞車感測器！(你剛剛漏了這行)
            self.just_found_target = True
                    
        else:
            # 🌟 觸發「尋敵成功瞬間」的電子急煞系統 (Auto-Brake)
            if getattr(self, 'just_found_target', False):
                
                # 🛑 做法改變：不再給反向煞車，而是徹底「拔掉電源」，讓速度與慣性瞬間歸零！
                self.last_action_yaw = 0.0      # 清空過去的物理慣性阻尼
                raw_action_yaw = 0.0            # 強制這一步的動作也是 0
                # ==========================================
                # 🛑 終極物理急煞：剪斷橡皮筋！
                # 瞬間把「虛擬目標角度」覆寫為「實體當前角度」
                # 這樣馬達的 PID 誤差會瞬間變成 0，產生無比強大的靜止煞車力！
                # ==========================================
                self.cmd_yaw = self.current_positions['yaw_joint']
                
                self.just_found_target = False
                
                # 印出一行 Log，確認觸發的是「歸零急煞」
                self.get_logger().info("🛑 [系統急煞] 發現目標！搖桿與慣性已瞬間強制歸 0，交接給 AI！")

        # ========================================================
        # 🦾 2. 核心防護：物理阻尼器 (Action Smoothing)
        # ========================================================
        alpha = 0.5
        action_yaw = (alpha * raw_action_yaw) + ((1.0 - alpha) * self.last_action_yaw)
        action_pitch = (alpha * raw_action_pitch) + ((1.0 - alpha) * self.last_action_pitch)

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
            # 🌟 經濟學修復：讓每一步的紅利高達 100 分！
            # 這樣 AI 算一算：留下來追蹤一步只能賺 60，開槍提早一步結束可以賺 100！
            # 它絕對會拼了命地一瞄準就立刻開火！
            time_bonus = max(0.0, (1500 - self.step_count) * 100.0)
            
            # 總得分 = 基礎 5000 + 巨額時間紅利
            total_hit_reward = 5000.0 + time_bonus
            reward += total_hit_reward
            
            self.get_logger().info(f"💥💥 命中！獲基礎 5000 + 紅利 {time_bonus:.0f} = {total_hit_reward:.0f} 分！")
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
        # 🌟 把 > 0.0 改成 > 0.8！必須要有超過 80% 的決心才允許開火，杜絕手抖走火！
        ai_wants_to_shoot = (action_shoot_trigger > 0.8)
        can_shoot = (current_time - self.last_shoot_time > 0.5)
        action_magnitude = (action_yaw**2 + action_pitch**2 + action_roll**2)

        if not is_lost:
            # 🟢 生存底薪與水平儀保護
            reward += 1.0 
            roll_error = abs(self.current_positions['roll_joint'] - (-self.roll_correction))
            if roll_error < 0.1:
                reward += 1.0

            # ==========================================
            # 🌟 新增：尋敵成功 / 重獲目標的「瞬間紅利」
            # 如果上一部是丟失狀態 (was_lost == True)，現在看到了，給予大獎勵！
            # ==========================================
            if self.was_lost:
                reward += 50.0
                status_str = "🎉 捕捉目標！獲得尋敵紅利！"
                self.get_logger().info("🎯 [雷達接觸] 成功在盲區中捕捉到目標！獲得尋敵紅利 +50 分！")

            # ==========================================
            # 🏆 1 & 2. 階梯式狀態機 + ABS 煞車 + 雷達鎖定
            # ==========================================
            if tracking_error_deg < 2.0:
                self.lock_on_counter += 1
                # 🌟 絕對紅心：唯一的暴富區
                reward += 60.0  
                status_str = "🎯 完美爆頭！(誤差 < 2°)"
                #reward -= action_magnitude * 5.0 
                #reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 1.0
                
            elif tracking_error_deg < 10.0:
                self.lock_on_counter += 1
                # 🌟 斷層 1：底薪大幅砍到 20，讓它眼紅 60 分
                reward += 50.0  
                status_str = "🟢 穩定鎖定 (誤差 < 10°)"

                raw_gravity = (tracking_error_deg / 2.0) ** 2
                gravity_penalty = min(15.0, raw_gravity) # 上限調低一點，避免跟外圍重疊
                reward -= gravity_penalty

                #reward -= action_magnitude * 1.0  
                #reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 1.0
                
            elif tracking_error_deg < 20.0:
                self.lock_on_counter = 0  
                # 🌟 斷層 2：底薪降到 5，幾乎被重力井抵銷，這裡是不賺不賠的掙扎區
                reward += 30.0   
                status_str = "🟡 邊緣捕捉 (誤差 < 20°)"

                raw_gravity = (tracking_error_deg / 4.0) ** 2
                gravity_penalty = min(30.0, raw_gravity)
                reward -= gravity_penalty

                #reward -= action_magnitude * 0.5  
                #reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 0.5
                
            else:
                self.lock_on_counter = 0  
                status_str = "🟠 視野邊緣危險區"
                # 🌟 斷層 3：拔除底薪！外圍就是純粹的痛苦，逼它趕快轉頭
                reward += 30.0 
                
                raw_gravity = (tracking_error_deg / 4.0) ** 2
                gravity_penalty = min(30.0, raw_gravity)
                reward -= gravity_penalty
                
                #reward -= action_magnitude * 0.5
                #reward -= ((action_yaw_diff**2) + (action_pitch_diff**2)) * 1.0

            # ==========================================
            # 🔫 3. 無限制開火 (完全交給 AI 自主決定)
            # ==========================================
            # 只要 AI 想開槍 (ai_wants_to_shoot)，且武器不在冷卻中 (can_shoot)
            if ai_wants_to_shoot and can_shoot:
                
                # 💥 沒有任何角度與時間限制，想開火就直接開火！
                self.shoot(force=15000.0)
                self.last_shoot_time = current_time
                
                # 💸 彈藥成本：打得越偏，後座力/浪費彈藥的懲罰越重
                aim_cost = tracking_error_deg * 10.0
                reward -= aim_cost+500
                
                # 根據射擊時的偏差，印出不同的 Log 狀態
                if tracking_error_deg < 10.0:
                    status_str = f"🔥 精準擊發！(偏差 {tracking_error_deg:.1f}°)"
                    self.get_logger().info(f"🎯 [開火] 偏差 {tracking_error_deg:.1f}°，精準發射！(微扣 {-aim_cost-500:.1f} 分)")
                else:
                    status_str = f"🔫 盲射開火！(偏差 {tracking_error_deg:.1f}°)"
                    self.get_logger().info(f"🔫 [盲射] 偏差 {tracking_error_deg:.1f}°，隨機開火！(重罰 {-aim_cost-500:.1f} 分)")
            
            # 如果瞄得很準，但 AI 選擇隱忍不發
            elif tracking_error_deg < 2.0 and not ai_wants_to_shoot:
                status_str = "🎯 完美鎖定！(等待 AI 扣板機)"

            self.was_lost = False

            # 🌟 1. 算出當前目標的「絕對世界角度」
            current_target_yaw = self.current_positions['yaw_joint'] - self.pan_cmd
            
            # 🌟 2. 計算兩幀之間的位置差異 (並處理 360 度折疊問題)
            diff = current_target_yaw - self.last_seen_pan
            shortest_diff = math.atan2(math.sin(diff), math.cos(diff))
            
            # 🌟 3. 算出瞬間角速度 (度/秒)
            raw_vel = shortest_diff / self.TIME_FROM_START
            
            # ==========================================
            # 🛑 核心防撞牆：物理極限速度裁切 (Velocity Clamping)
            # 雙體船的角速度不可能超過 0.5 rad/s (約每秒 28 度)。
            # 如果超過這個值，絕對是「相機自己猛轉」造成的視覺殘影！強制砍掉！
            # ==========================================
            raw_vel = np.clip(raw_vel, -0.5, 0.5)

            # 🌟 4. 用平滑濾波(EMA)降低視覺雜訊，更新記憶
            self.last_seen_yaw_vel = 0.8 * self.last_seen_yaw_vel + 0.2 * raw_vel
            
            self.last_seen_pan = current_target_yaw
            self.last_seen_tilt = self.current_positions['pitch_joint'] + self.tilt_cmd
            
        else:
            # ==========================================
            # 🧭 3. 丟失目標 (全域探索與記憶尋跡)
            # ==========================================
            self.was_lost = True

            # 🌟 準備一個附加警告的字串
            pitch_warn = ""

            # 🔴 [懲罰] 仰角安全邊界 (避免尋敵時看天看地)
            pitch_pos = self.current_positions['pitch_joint']
            if abs(pitch_pos) > 0.6:  
                pitch_penalty = ((abs(pitch_pos) - 0.6) * 10.0) ** 2
                reward -= pitch_penalty
                pitch_warn = " ⚠️ 仰角大扣血!" # 記錄警告，等一下接在後面

            # ==========================================
            # 🌟 核心修復：根據「是否看過目標」切換戰術！
            # ==========================================
            
            # 🚨 新增防護機制：預測保質期 (Timeout)
            if self.has_seen_target and time_since_vision > 1.5:
                # 1. 反轉未來的盲掃方向 (原本往右找，現在改往左找！)
                self.search_spin_dir = -self.search_spin_dir 
                
                # 2. 強制拔除「看過目標」的標籤
                self.has_seen_target = False
                
                self.get_logger().info("⏳ [戰術切換] 預測超時 (丟失 > 1.5s)，放棄幽靈，反向慢速迴轉掃描！")

            if self.has_seen_target:
                # 🟢 戰術 A：【動態預測尋跡模式】(看過但溜走了，且在 1.5 秒內)
                status_str = f"🧭 預測追跡中...{pitch_warn}"
                
                panic_penalty = 0 + (time_since_vision * 2.0)
                reward -= panic_penalty

                # ==========================================
                # 🌟 幽靈阻力 (Velocity Decay)：讓預測速度隨時間衰減
                # 剛丟失 (0秒) 時保留 100%，1.5 秒時衰減到 0%
                # ==========================================
                decay_factor = max(0.0, 1.0 - (time_since_vision / 1.5))
                decayed_vel = self.last_seen_yaw_vel * decay_factor

                # 使用「衰減後的速度」來計算預測位置
                predicted_pan = self.last_seen_pan + (decayed_vel * time_since_vision)

                yaw_diff_rad = self.current_positions['yaw_joint'] - predicted_pan
                shortest_yaw_diff_rad = math.atan2(math.sin(yaw_diff_rad), math.cos(yaw_diff_rad))
                predicted_pan_diff_deg = abs(math.degrees(shortest_yaw_diff_rad))

                memory_penalty = predicted_pan_diff_deg * 0.6
                reward -= memory_penalty

            else:
                # 🔴 戰術 B：【全頻雷達掃描模式】(開局在盲區，還沒看到人)
                status_str = f"📡 盲掃中...{pitch_warn}"
                
                # ==========================================
                # 🌟 1. 瞎眼底薪 (加重！絕不允許淨賺！)
                # ==========================================
                # 固定底薪扣 10 分，加上隨時間增加的焦慮感
                panic_penalty = 10.0 + (time_since_vision * 4.0)
                reward -= panic_penalty
                
                # ==========================================
                # 🌟 2. 專治「點頭娃娃」：完全鎖死 Pitch！
                # 盲掃時強迫視線貼齊海平面，不准上下看！
                # ==========================================
                #reward -= (self.current_positions['pitch_joint'] ** 2) * 10.0
                #reward -= (action_pitch ** 2) * 5.0
                
                # ==========================================
                # 🌟 3. 專治「人體雨刷」：零容忍方向法規
                # ==========================================
                #if abs(self.last_action_yaw) > 0.1 and (action_yaw * self.last_action_yaw) < -0.1:
                #    reward -= 20.0
                #    status_str = "📡 盲掃中... ⚠️ 嚴禁左右橫跳!"
                
                # ==========================================
                # 🌟 4. 鼓勵極速迴轉 (「減刑」而非「獎勵」)
                # ==========================================
                #if abs(action_yaw) < 0.3:
                #    reward -= 10.0  # 怠惰懲罰：轉太慢再多扣 10 分
                #else:
                    # 💡 只有同方向轉動，才能稍微抵銷「瞎眼底薪」
                #    if (action_yaw * self.last_action_yaw) >= 0:
                #        reward += (abs(action_yaw) * 5.0)

        
                
        # ==========================================
        # 📊 4. 狀態輸出與強制結束機制
        # ==========================================
        if self.step_count % 5 == 0:
            self.get_logger().info(
                f"[{status_str}] Err: {tracking_error_deg:.2f}° | Reward: {reward:.2f} | Act: [{action_yaw:.2f}, {action_pitch:.2f}]"
            )

        truncated = False
        if self.step_count >= 1500:
            truncated = True
            self.get_logger().info("⏳ 追蹤時間耗盡 (已達 1500 步)，強制重新佈局！")

        timeout_seconds = 5.0 
        if time_since_vision > timeout_seconds:
            terminated = True
            
            # ==========================================
            # 🌟 終局結算：區分「中途追丟」與「徹底失明」
            # ==========================================
            if not self.has_seen_target:
                # 💀 死法 A：從頭到尾都沒看到目標 (掃描戰術徹底失敗)
                reward -= 200.0  
                self.get_logger().info(f"盲💀 徹底失明！開局 5 秒內未尋獲目標，重罰 -200 分！強制重置！")
            else:
                # 💀 死法 B：曾經鎖定過，但後來被甩開超過 5 秒 (追蹤戰術失敗)
                reward -= 100.0  
                self.get_logger().info(f"💀 追丟超時 ({timeout_seconds:.1f} 秒)！獲得懲罰 -100，強制重置世界！")


        # ==========================================
        # 🌟 核心修復：在結算完所有的獎勵懲罰後，才更新「最後動作」的記憶
        # ==========================================
        self.last_action_yaw = action_yaw
        self.last_action_pitch = action_pitch

        obs = self._get_obs()
        
        #target_time = 0.005  
        #step_execution_time = time.time() - current_time
        #if step_execution_time < target_time:
        #    time.sleep(target_time - step_execution_time)
        
        self.last_step_time = time.time()
        
        # ==========================================
        # 💊 降血壓特效藥：將總分縮小 100 倍！
        # 讓終端機保留大數字給人類看，但餵給 AI 的是縮小後的安全數值
        # ==========================================
        scaled_reward = reward / 1000.0
        
        # 🌟 注意最後這裡回傳的是 scaled_reward
        return obs, scaled_reward, terminated, truncated, {}