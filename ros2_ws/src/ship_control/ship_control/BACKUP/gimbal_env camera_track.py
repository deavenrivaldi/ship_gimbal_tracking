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
    # 🌟 新增這個變數：視覺丟失容忍時間（秒）
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

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        
        self.last_seen_pan = 0.0
        self.last_seen_tilt = 0.0
        self.step_count = 0
        self.max_steps = 1000
        self.smoothed_roll = 0.0

        # 🌟 加入回合計數器
        self.episode_count = 0
        
        self.last_step_time = time.time()
        self.was_lost = True 
        # 🌟 加入：船隻動力變換計時器
        self.thrust_timer = 0



        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Fire service client
        self.fire_client = self.create_client(Fire, '/fire')



        
        # 🌟 新增：用來記錄這回合「是否已經看過目標了」
        self.has_seen_target = False
        
        self.get_logger().info('✅ Gimbal RL Environment Ready!')
    



    def get_camera_world_pose(self):
    #"""Get camera world pose from TF2."""
        try:
            return self.tf_buffer.lookup_transform(
                'gimbal_world',
                'gimbal_wam_v/gimbal/camera_link',
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warning(f'TF2 lookup failed: {e}', throttle_duration_sec=1.0)
            return None

    def quaternion_to_direction(self, q) -> Vector3:
        #"""Convert camera orientation quaternion to world direction vector."""
        direction = Vector3()
        direction.x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        direction.y = 2.0 * (q.x * q.y + q.w * q.z)
        direction.z = 2.0 * (q.x * q.z - q.w * q.y)
        return direction

    def shoot(self, force=15000.0):
        #"""Fire a projectile from camera position in camera direction."""
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
        
        # 只要 YOLO 傳來資料，代表我們看到目標了！
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
        # 🌟 把 0.3 換成 self.VISION_TIMEOUT
        is_lost = 1.0 if (time_since_vision > self.VISION_TIMEOUT or not self.has_seen_target) else 0.0
        
        normalized_yaw = math.atan2(math.sin(self.current_positions['yaw_joint']), math.cos(self.current_positions['yaw_joint']))
        
        # ========================================================
        # 🧭 核心修正：計算「實體記憶羅盤」誤差並餵給神經網路！
        # 讓大腦直接看到「最短路徑的偏誤角度」，它才知道該往哪邊回頭！
        # ========================================================
        yaw_diff_rad = self.current_positions['yaw_joint'] - self.last_seen_pan
        # 這個 shortest_yaw_diff 會是一個帶正負號的弧度，完美指示最短回頭方向
        shortest_yaw_diff = math.atan2(math.sin(yaw_diff_rad), math.cos(yaw_diff_rad))
        
        tilt_diff = self.current_positions['pitch_joint'] - self.last_seen_tilt
        
        return np.array([
            self.pan_cmd, self.tilt_cmd, 
            self.roll_correction, self.pitch_correction,
            normalized_yaw,  
            self.current_positions['pitch_joint'], self.current_positions['roll_joint'],
            is_lost,
            self.last_seen_pan,
            time_since_vision * 0.1,
            # 🌟 新增的兩個超級導航感測器：
            shortest_yaw_diff, 
            tilt_diff
        ], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0

        try:
            reset_cmd = f"gz service -s /world/{self.world_name}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'reset: {{all: true}}'"
            subprocess.run(reset_cmd, shell=True, capture_output=True, text=True)
            
            # 給物理引擎一點點時間歸零
            time.sleep(0.1)

            # 🌟 回合數 +1
            self.episode_count += 1
            # ========================================================
            # 🌟 課程學習空投機制 (Curriculum Spawn)
            # ========================================================
            if self.episode_count <= 50:
                # 前 50 回合：強制把大船空投在雲台的「正後方」(X為負數)
                # 假設雲台在原點 (0,0)，我們把船放在 X: -8 ~ -15 的位置
                rand_x = random.uniform(-12.0, -8.0)
                # Y 軸給一點點隨機偏移，讓它不至於每次都在死正後方
                rand_y = random.uniform(-3.0, 3.0) 
                self.get_logger().info(f"🎓 [魔鬼特訓] 第 {self.episode_count} 回合：目標空投於正後方！")
            else:
                # 50 回合後：恢復正常的全域隨機空投
                center_x = 8.0  
                center_y = 0.0  
                min_r = 8.0
                max_r = 15.0
                r = random.uniform(min_r, max_r)
                theta = random.uniform(-math.pi, math.pi)
                rand_x = center_x + r * math.cos(theta)
                rand_y = center_y + r * math.sin(theta)
            
            # 船的隨機朝向維持不變
            rand_yaw = random.uniform(-math.pi, math.pi) 
            qw = math.cos(rand_yaw / 2.0)
            qz = math.sin(rand_yaw / 2.0)

            # 🌟 空投機制：將 Z 座標設為 3.0，讓大船從空中掉進水裡，徹底解決穿模翻船問題！
            set_pose_cmd = (
                f"gz service -s /world/{self.world_name}/set_pose "
                f"--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--req 'name: \"wam-v\", position: {{x: {rand_x}, y: {rand_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'"
            )
            subprocess.run(set_pose_cmd, shell=True, capture_output=True, text=True)

            # 3. 給予大船隨機推力
            time.sleep(0.02)
            base_t = random.uniform(-2, 4)*100  
            diff_t = random.uniform(-1.5, 1.5)*100  
            thrust_cmd = f"gz topic -t /model/wam-v/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t + diff_t}\" & " \
                         f"gz topic -t /model/wam-v/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t - diff_t}\""
            subprocess.run(thrust_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            self.get_logger().info(f"🚤 世界重置！大船空投於 ({rand_x:.1f}, {rand_y:.1f})")

        except Exception as e:
            self.get_logger().error(f"重置失敗: {e}")
            
        #self.steps_since_last_vision = 100
        self.pan_cmd = 0.0
        self.tilt_cmd = 0.0
        self.kf_initialized = False 
        
        # 🌟 拔除雷達作弊：讓初始記憶歸零，AI 必須自己轉頭找！
        self.last_seen_pan = 0.0  
        self.last_seen_tilt = 0.0   
        self.has_seen_target = False  # 標記為「尚未發現目標」
        
        positions = {'yaw_joint': 0.0, 'pitch_joint': 0.0, 'roll_joint': 0.0}
        self.pub_trajectory.publish(self.build_trajectory(positions, 0.1))
        
        # 等待空投船隻落水與軌跡歸零
        time.sleep(0.3)
        
        # 🌟 核心修正：開局時間直接同步為「目前時間」，不要減去 100 秒！
        self.last_vision_time = time.time()  
    
        self.was_lost = True
        return self._get_obs(), {}
        

    def step(self, action):
        current_time = time.time()
        time_gap = current_time - self.last_step_time
        
        # =====================================================================
        # 🏎️ 終極時空修復：時間軸防塌陷補償鎖 (Time Compensation Lock)
        # 正常訓練下每步間隔不到 0.005 秒。若超過 0.15 秒，代表 PPO 剛才停下來生成 Report 並更新權重了。
        # 我們不再重置世界，而是將這段「思考時間」等量加回 self.last_vision_time 中！
        # 對大腦而言，更新權重的這段時間相當於「時空靜止」，醒來後能完美銜接狀態，繼續探索！
        # =====================================================================
        if time_gap > 0.3:
            self.get_logger().info(f"🧠 神經網路更新完畢（耗時 {time_gap:.2f} 秒）！已自動補正時間軸，繼續探索任務。")
            
            # 🌟 核心關鍵：將大腦思考的時間，等量補償給最後看到目標的時間
            self.last_vision_time += time_gap  
            
            # 同步重設上一步時間，防止時空錯亂
            self.last_step_time = current_time

        # 恢復正常的步伐計數
        self.step_count += 1

        # =====================================================================
        # 🌊 動態海象干擾系統 (Dynamic Thrust Disturbance)
        # 🌟 就是要貼在這裡！讓它跟著 step 每 0.05 秒跳動一次！
        # =====================================================================
        self.thrust_timer += 1
        if self.thrust_timer >= 60:
            self.thrust_timer = 0
            # 產生新的隨機推力
            base_t = random.uniform(-3, 5) * 100  
            diff_t = random.uniform(-2.5, 2.5) * 100  
            
            thrust_cmd = f"gz topic -t /model/wam-v/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t + diff_t}\" & " \
                         f"gz topic -t /model/wam-v/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p \"data: {base_t - diff_t}\""
            
            subprocess.Popen(thrust_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # (已經幫你修好縮排了)
            self.get_logger().info(f"🌊 海流變化！船隻切換動力：前進 {base_t:.0f}, 轉向 {diff_t:.0f}")

        action_yaw = float(action[0])
        action_pitch = float(action[1])

        
        # =====================================================================
        # 🎯 核心修正 1：計算當前狀態與視覺追蹤誤差 (提前計算，供排檔系統使用)
        # =====================================================================
        # 🌟 把 0.3 換成 self.VISION_TIMEOUT
        time_since_vision = time.time() - self.last_vision_time 
        is_lost = (time_since_vision > self.VISION_TIMEOUT) or (not self.has_seen_target)
        
        pan_err = self.pan_cmd if not is_lost else 0.0
        tilt_err = self.tilt_cmd if not is_lost else 0.0
        tracking_error = math.sqrt(pan_err**2 + tilt_err**2)
        tracking_error_deg = math.degrees(tracking_error)

        # =====================================================================
        # 🏎️ 核心修正 2：將「自動排檔系統」搬回正確的控制路徑 (動作輸出前)
        # =====================================================================
        if not is_lost:
            if tracking_error_deg < 2.0:
                gear_ratio = 0.005   # 🎯 紅心區 (一檔：微調懸停)
            elif tracking_error_deg < 8.0:
                gear_ratio = 0.005   # 🟢 內圈區 (二檔：平滑靠近)
            elif tracking_error_deg < 20.0:
                gear_ratio = 0.008   # 🟡 邊緣區 (三檔：極速追趕)
            else:
                gear_ratio = 0.01   # 🟠 視野外邊緣 (四檔：油門全開拉回)
                
            action_yaw *= gear_ratio
            action_pitch *= gear_ratio
        
        # =========================================================
        # 🏎️ 大腦升速解鎖！
        # 從 0.20 拉高到 0.50 (約每秒 570 度)！這絕對跟得上船了！
        # 但是我們依舊配合 smoothness 懲罰來防抖。
        # =========================================================
        max_yaw_step = 0.15   
        max_pitch_step = 0.10  

        yaw_target = self.current_positions['yaw_joint'] - (action_yaw * max_yaw_step)
        pitch_target = self.current_positions['pitch_joint'] + (action_pitch * max_pitch_step)

        # 🌟 計算「搖桿抖動程度」（現在的動作減去上一步的動作）
        action_yaw_diff = action_yaw - self.last_action_yaw
        action_pitch_diff = action_pitch - self.last_action_pitch
        
        # 更新肌肉記憶
        self.last_action_yaw = action_yaw
        self.last_action_pitch = action_pitch

        alpha = 0.1 
        self.smoothed_roll = (1.0 - alpha) * self.smoothed_roll + alpha * self.roll_correction

        yaw_target = np.clip(yaw_target, self.JOINT_LIMITS['yaw_joint'][0], self.JOINT_LIMITS['yaw_joint'][1])
        
        # 只保留 Pitch 的 clip
        pitch_target = np.clip(pitch_target, self.JOINT_LIMITS['pitch_joint'][0], self.JOINT_LIMITS['pitch_joint'][1])
        
        roll_target = self.smoothed_roll 

        self.current_positions['yaw_joint'] = yaw_target
        self.current_positions['pitch_joint'] = pitch_target
        self.current_positions['roll_joint'] = roll_target

        positions = {'yaw_joint': yaw_target, 'pitch_joint': pitch_target, 'roll_joint': roll_target}
        self.pub_trajectory.publish(self.build_trajectory(positions, self.TIME_FROM_START))

        # =========================================================================
        # 🌟 圓周最短路徑修正 (Shortest Path on a Circle)
        # 必須使用 atan2 來計算無限旋轉關節的真實角度差，否則 360 度會被誤判為極大誤差！
        # =========================================================================
        yaw_diff_rad = self.current_positions['yaw_joint'] - self.last_seen_pan
        shortest_yaw_diff_rad = math.atan2(math.sin(yaw_diff_rad), math.cos(yaw_diff_rad))
        last_seen_pan_diff_deg = abs(math.degrees(shortest_yaw_diff_rad))

        # Pitch 維持直接相減 (因為 Pitch 不能無限轉)
        last_seen_tilt_diff_deg = abs(math.degrees(self.current_positions['pitch_joint']) - math.degrees(self.last_seen_tilt))
        
        last_seen_pose_diff_deg = math.sqrt(last_seen_pan_diff_deg**2 + last_seen_tilt_diff_deg**2)

        # ==========================================
        # 🎯 4. Reward Function (真・盲搜與防呆機制)
        # ==========================================
        reward = 0.0


        if not is_lost:
            status_str = "👀 AI 視覺鎖定中"
            reward += 10.0
            

            # ========================================================
            # 🎯 核心修正 1：引力漏斗 (連續性距離獎勵)
            # 只要在畫面內 (假設最大誤差約 30度)，越靠近中心分數越高。
            # 這給大腦一個明確的「斜坡」，引導它往中心爬。
            # ========================================================
            funnel_bonus = max(0.0, 30.0 - tracking_error_deg) * 0.5
            reward += funnel_bonus

            # ========================================================
            # 🎯 3. 同心圓階梯獎勵 & 動態煞車
            # ========================================================
            action_magnitude = (action_yaw**2 + action_pitch**2)
            
            if tracking_error_deg < 2.0:
                reward += 60.0  
                status_str = "🎯 完美爆頭！(誤差 < 2°)"
                reward -= action_magnitude * 15.0 
            
            elif tracking_error_deg < 8.0:
                reward += 20.0  
                status_str = "🟢 穩定追蹤 (誤差 < 8°)"
                reward -= action_magnitude * 5.0  
            elif tracking_error_deg < 20.0:
                reward += 5.0   
                status_str = "🟡 邊緣捕捉 (誤差 < 20°)"
                reward -= action_magnitude * 1.0  
            else:
                status_str = "🟠 視野邊緣危險區"
                pass

            # =========================================================================
            # 🌟 4. 回頭大獎機制 (從記憶回歸中成功找回的獎勵)
            # =========================================================================
            if self.was_lost:
                if self.step_count > 5:
                    guiding_bonus = max(0.0, 50.0 - last_seen_pose_diff_deg)
                    reward += guiding_bonus
                    if guiding_bonus > 10.0:
                         self.get_logger().info(f"🎉 🎉 🎉 AI 成功找回目標！獲得大獎 +{guiding_bonus:.1f}！")
                self.was_lost = False

            # =========================================================================
            # 🌟 5. 刷新記憶錨點 (準備給下一次追丟時使用)
            # =========================================================================
            self.last_seen_pan = self.current_positions['yaw_joint'] - self.pan_cmd
            self.last_seen_tilt = self.current_positions['pitch_joint'] + self.tilt_cmd
            
        else:
            self.was_lost = True
            
            panic_penalty = min(30.0, 2.0 + (time_since_vision * 2.0))
            reward -= panic_penalty

            if self.has_seen_target:
                status_str = "🔍 AI 記憶回歸中"
                

                
                # ========================================================
                # 🌟 核心修正：加入「水平線維持獎勵/懲罰」
                # 限制它不能在追蹤時死命低頭。
                # 超過 0.8 rad (約 45 度) 的俯角時，開始給予懲罰。
                # 這能迫使 AI 學會：在能看見船的前提下，盡量維持雲台水平。
                # ========================================================
                pitch_pos = self.current_positions['pitch_joint']
                if abs(pitch_pos) > 0.6:
                    # 超過 45 度後，越低頭懲罰越重，逼它想辦法把船留在中間
                    reward -= (abs(pitch_pos) - 0.6) * 4.0 + 5
            else:
                status_str = "🧭 開局全域盲搜中"
                
                # (1) 嚴禁看天空或地板
                if abs(self.current_positions['pitch_joint']) > 1.0:
                    reward -= 15.0
                # ========================================================
                # 🌟 (2) 新增：怠惰懲罰 (Laziness Penalty) - 嚴禁待在原地發呆！
                # 盲搜階段必須不斷轉頭！如果大腦的推桿總出力太小，就狠狠扣分。
                # 結合下面的防抖懲罰，大腦唯一的出路就是「死死往同一個方向推到底」！
                # ========================================================
                action_magnitude = math.sqrt(action_yaw**2 + action_pitch**2)
                if action_magnitude < 0.5:
                    # 出力越接近 0，扣分越重 (最高每步扣 5 分)
                    laziness_penalty = (0.5 - action_magnitude) * 10.0
                    reward -= laziness_penalty

                # ========================================================
                # (3) 平滑懲罰 (Smoothness Penalty)
                # 嚴厲懲罰「來回狂抖搖桿」的行為。
                # ========================================================
                jitter_penalty = ((action_yaw_diff**2) + (action_pitch_diff**2)) * 2.0
                reward -= jitter_penalty
                
                
                
        if self.step_count % 5 == 0:
            self.get_logger().info(
                f"[{status_str}] Err: {tracking_error_deg:.2f}° | Reward: {reward:.2f} | Act: [{action_yaw:.2f}, {action_pitch:.2f}]"
            )

        terminated = False
        
        # 🌟 還原快速重製時間點（完美對應原本的 2047 步與 1200 步速率）：
        # 開局全域盲搜：給予 6.0 秒找不到就重製（對應原本開局沒見過的寬限期）
        # 中途追丟目標：只要連續超過 0.8 秒沒看到，立刻果斷重製（還原原本 1200 步快速洗數據的節奏）
        timeout_seconds = 6.0 

        if time_since_vision > timeout_seconds:
            reward -= 100.0  
            terminated = True
            self.get_logger().info(f"💀 追丟超時 ({timeout_seconds:.2f} 秒)！獲得懲罰 -100，強制重置世界！")

        obs = self._get_obs()
        truncated = False  

        # ========================================================
        # 🌟 變速齒輪：調整 target_time 來控制訓練播放速度
        # 0.05 = 1倍速 (真實時間)
        # 0.02 = 2.5倍速 (推薦！兼顧觀賞與訓練速度)
        # 0.01 = 5倍速 (極快，但肉眼還勉強跟得上)
        # ========================================================
        target_time = 0.005  # 你可以隨時修改這個數字來換檔
        
        step_execution_time = time.time() - current_time
        if step_execution_time < target_time:
            time.sleep(target_time - step_execution_time)
        
        self.last_step_time = time.time()
        
        return obs, reward, terminated, truncated, {}