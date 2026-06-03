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


class TargetKalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 10.0 
        self.F = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.R = np.array([[0.005, 0], [0, 0.005]])
        q = 0.05
        self.Q = np.array([
            [q*(dt**3)/3, q*(dt**2)/2, 0, 0],
            [q*(dt**2)/2, q*dt,        0, 0],
            [0, 0, q*(dt**3)/3, q*(dt**2)/2],
            [0, 0, q*(dt**2)/2, q*dt       ]
        ])

    def predict(self, is_lost=False):
        decay = 0.85 if is_lost else 1.0
        self.F[1, 1] = decay
        self.F[3, 3] = decay
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update(self, z):
        y = z - (self.H @ self.x) 
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S) 
        self.x = self.x + (K @ y)
        self.P = (np.eye(4) - (K @ self.H)) @ self.P

class ShipGimbalEnv(gym.Env, Node):
    JOINT_NAMES = ['yaw_joint', 'pitch_joint', 'roll_joint']
    JOINT_LIMITS = {
        'yaw_joint':   (-1000, 1000),
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
        self.kf = TargetKalmanFilter(dt=self.TIME_FROM_START)
        self.kf_initialized = False

        self.sub_vision = self.create_subscription(Vector3, '/gimbal/angle_command', self.vision_callback, 10)
        self.sub_imu = self.create_subscription(Vector3, '/gimbal/roll_correction', self.imu_correction_callback, 10)
        self.sub_joints = self.create_subscription(JointState, f'/world/{self.world_name}/model/gimbal/joint_state', self.joint_state_callback, 10)
        self.pub_trajectory = self.create_publisher(JointTrajectory, '/gimbal/joint_trajectory', 10)

        # 動作空間：[Yaw控制, Pitch控制, Roll控制, 射擊觸發器]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        # 觀察空間：14維度，包含目標移動速度
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(14,), dtype=np.float32)
        
        self.last_pan_err_obs = 0.0
        self.last_tilt_err_obs = 0.0

        self.last_seen_pan = 0.0
        self.last_seen_tilt = 0.0
        self.step_count = 0
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
        
        # 🌟 已更新為獨立 person_link 的精確感測器名稱
        self.sub_hit = self.create_subscription(
            Contacts, 
            '/world/gimbal_world/model/wamv/link/person_link/sensor/person_contact/contact', 
            self.hit_callback, 
            10
        )

        self.has_seen_target = False
        self.get_logger().info('✅ Gimbal RL Environment Ready!')
    
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

    # 🌟 核心修正 1：預設力道從 15000 降至 1500，防止量子隧穿效應穿模
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

        return np.array([
            self.pan_cmd, self.tilt_cmd, 
            self.roll_correction, self.pitch_correction,
            normalized_yaw,  
            self.current_positions['pitch_joint'], self.current_positions['roll_joint'],
            is_lost,
            self.last_seen_pan,
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
            
            if self.episode_count <= 50:
                rand_x = random.uniform(0.0, -4.0)
                rand_y = random.uniform(-1.0, 1.0) 
                self.get_logger().info(f"🎓 [魔鬼特訓] 第 {self.episode_count} 回合：目標空投於正後方！")
            else:
                center_x = 8.0  
                center_y = 0.0  
                min_r = 8.0
                max_r = 15.0
                r = random.uniform(min_r, max_r)
                theta = random.uniform(-math.pi, math.pi)
                rand_x = center_x + r * math.cos(theta)
                rand_y = center_y + r * math.sin(theta)
            
            rand_yaw = random.uniform(-math.pi, math.pi) 
            qw = math.cos(rand_yaw / 2.0)
            qz = math.sin(rand_yaw / 2.0)

            # ========================================================
            # 🌟 雙船同步重置：先把開走的主船硬抓回初始原點 (8, 0, 0.2)
            # ========================================================
            reset_main_ship_cmd = (
                f"gz service -s /world/{self.world_name}/set_pose "
                f"--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--req 'name: \"gimbal_wam_v\", position: {{x: 8.0, y: 0.0, z: 0.2}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'"
            )
            subprocess.run(reset_main_ship_cmd, shell=True, capture_output=True, text=True)
            
            # 給物理引擎 20 毫秒的喘息時間定形
            time.sleep(0.02)

            # ========================================================
            # 接著再執行你原本的目標船空投 (這時相對距離才會100%正確！)
            # ========================================================
            set_pose_cmd = (
                f"gz service -s /world/{self.world_name}/set_pose "
                f"--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--req 'name: \"wamv\", position: {{x: {rand_x}, y: {rand_y}, z: 0.2}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'"
            )
            subprocess.run(set_pose_cmd, shell=True, capture_output=True, text=True)

            time.sleep(0.02)
            base_t = random.uniform(-2, 4)*100  
            diff_t = random.uniform(-1.5, 1.5)*100  
            thrust_cmd = f"gz topic -t /model/wamv/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t + diff_t}\" & " \
                         f"gz topic -t /model/wamv/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t - diff_t}\""
            subprocess.run(thrust_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            self.get_logger().info(f"🚤 世界重置！大船空投於 ({rand_x:.1f}, {rand_y:.1f})")

        except Exception as e:
            self.get_logger().error(f"重置失敗: {e}")
            
        self.pan_cmd = 0.0
        self.tilt_cmd = 0.0
        self.kf_initialized = False 
        
        self.last_seen_pan = 0.0  
        self.last_seen_tilt = 0.0   
        self.has_seen_target = False  
        
        # 🌟 核心修正 2：每次回合結束，徹底清空測速槍的記憶，防止產生瞬間移動假速度炸毀權重
        self.last_pan_err_obs = 0.0
        self.last_tilt_err_obs = 0.0
        
        positions = {'yaw_joint': 0.0, 'pitch_joint': 0.0, 'roll_joint': 0.0}
        self.pub_trajectory.publish(self.build_trajectory(positions, 0.1))
        
        time.sleep(0.3)
        self.last_vision_time = time.time()  
        self.was_lost = True

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.01)

        return self._get_obs(), {}

    def hit_callback(self, msg):
        # 🌟 核心修正 3：必須是 msg.contacts (有 's')，否則 ROS 2 會當機
        for contact in msg.contacts:
            col1 = contact.collision1.name
            col2 = contact.collision2.name
            
            if 'projectile' in col1 or 'projectile' in col2:
                self.physical_hit = True
                break

    def step(self, action):
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.002)

        current_time = time.time()
        time_gap = current_time - self.last_step_time
        
        if time_gap > 0.3:
            self.get_logger().info(f"🧠 神經網路更新完畢（耗時 {time_gap:.2f} 秒）！已自動補正時間軸，繼續探索任務。")
            self.last_vision_time += time_gap  
            self.last_step_time = current_time

        self.step_count += 1

        self.thrust_timer += 1
        if self.thrust_timer >= 60:
            self.thrust_timer = 0
            base_t = random.uniform(-3, 5) * 100  
            diff_t = random.uniform(-2.5, 2.5) * 100  
            
            thrust_cmd = f"gz topic -t /model/wamv/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t + diff_t}\" & " \
                         f"gz topic -t /model/wamv/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t - diff_t}\""
            
            subprocess.Popen(thrust_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"🌊 海流變化！船隻切換動力：前進 {base_t:.0f}, 轉向 {diff_t:.0f}")

        action_yaw = float(action[0])
        action_pitch = float(action[1])
        action_roll = float(action[2])
        action_shoot_trigger = float(action[3]) 

        time_since_vision = time.time() - self.last_vision_time 
        is_lost = (time_since_vision > self.VISION_TIMEOUT) or (not self.has_seen_target)
        
        pan_err = self.pan_cmd if not is_lost else 0.0
        tilt_err = self.tilt_cmd if not is_lost else 0.0
        tracking_error = math.sqrt(pan_err**2 + tilt_err**2)
        tracking_error_deg = math.degrees(tracking_error)

        if not is_lost:
            if tracking_error_deg < 2.0:
                gear_ratio = 0.005
            elif tracking_error_deg < 8.0:
                gear_ratio = 0.005
            elif tracking_error_deg < 20.0:
                gear_ratio = 0.008
            else:
                gear_ratio = 0.01
                
            action_yaw *= gear_ratio
            action_pitch *= gear_ratio
        
        max_yaw_step = 0.15   
        max_pitch_step = 0.10 
        max_roll_step = 0.10

        yaw_target = self.current_positions['yaw_joint'] - (action_yaw * max_yaw_step)
        pitch_target = self.current_positions['pitch_joint'] + (action_pitch * max_pitch_step)
        
        # 🌟 保持這塊新手輔助輪：完全沒收大腦 Roll 的控制權，強制鎖平水線
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
        
        if self.physical_hit:
            reward += 1000.0
            self.get_logger().info("💥💥 物理碰撞確認！子彈命中目標，獲得 1000 分！")
            self.physical_hit = False

        ai_wants_to_shoot = (action_shoot_trigger > 0.0)
        can_shoot = (current_time - self.last_shoot_time > 1.0)

        if not is_lost:
            status_str = "👀 AI 視覺鎖定中"
            reward += 10.0

            funnel_bonus = max(0.0, 30.0 - tracking_error_deg) * 0.5
            reward += funnel_bonus

            roll_error = abs(self.current_positions['roll_joint'] - self.roll_correction)
            if roll_error < 0.1:
                reward += 2.0

            action_magnitude = (action_yaw**2 + action_pitch**2 + action_roll**2)

            if ai_wants_to_shoot and can_shoot:
                reward -= 2.0  
                status_str = "🔥 AI 擊發子彈！(等待物理碰撞...)"
                self.shoot(force=15000.0) # 🌟 確認這裡也降到了安全的力道
                self.last_shoot_time = current_time

            elif tracking_error_deg < 2.0:
                reward += 10.0  
                reward -= action_magnitude * 15.0 
                if not ai_wants_to_shoot:
                     status_str = "🎯 完美鎖定 (等待 AI 開火...)"
            
            elif tracking_error_deg < 8.0:
                reward += 5.0  
                reward -= action_magnitude * 5.0  
            elif tracking_error_deg < 20.0:
                reward += 2.0   
                reward -= action_magnitude * 1.0  

            if self.was_lost:
                if self.step_count > 5:
                    guiding_bonus = max(0.0, 50.0 - last_seen_pose_diff_deg)
                    reward += guiding_bonus
                self.was_lost = False

            self.last_seen_pan = self.current_positions['yaw_joint'] - self.pan_cmd
            self.last_seen_tilt = self.current_positions['pitch_joint'] + self.tilt_cmd
            
        else:
            self.was_lost = True
            
            panic_penalty = min(30.0, 2.0 + (time_since_vision * 2.0))
            reward -= panic_penalty

            if self.has_seen_target:
                status_str = "🔍 AI 記憶回歸中"
                
                pitch_pos = self.current_positions['pitch_joint']
                if abs(pitch_pos) > 0.6:
                    reward -= (abs(pitch_pos) - 0.6) * 4.0 + 5
            else:
                status_str = "🧭 開局全域盲搜中"
                
                if abs(self.current_positions['pitch_joint']) > 1.0:
                    reward -= 15.0
                
                action_magnitude = math.sqrt(action_yaw**2 + action_pitch**2)
                if action_magnitude < 0.5:
                    laziness_penalty = (0.5 - action_magnitude) * 10.0
                    reward -= laziness_penalty

                jitter_penalty = ((action_yaw_diff**2) + (action_pitch_diff**2)) * 2.0
                reward -= jitter_penalty
                
        if self.step_count % 5 == 0:
            self.get_logger().info(
                f"[{status_str}] Err: {tracking_error_deg:.2f}° | Reward: {reward:.2f} | Act: [{action_yaw:.2f}, {action_pitch:.2f}]"
            )

        terminated = False
        
        timeout_seconds = 6.0 

        if time_since_vision > timeout_seconds:
            reward -= 100.0  
            terminated = True
            self.get_logger().info(f"💀 追丟超時 ({timeout_seconds:.2f} 秒)！獲得懲罰 -100，強制重置世界！")

        obs = self._get_obs()
        truncated = False  

        target_time = 0.005  
        
        step_execution_time = time.time() - current_time
        if step_execution_time < target_time:
            time.sleep(target_time - step_execution_time)
        
        self.last_step_time = time.time()
        
        return obs, reward, terminated, truncated, {}