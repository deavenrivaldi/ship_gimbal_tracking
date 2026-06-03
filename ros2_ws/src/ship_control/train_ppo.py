import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from stable_baselines3 import PPO

# 引入剛剛建立好的環境類別
from gimbal_env import ShipGimbalEnv
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

class CurriculumCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(CurriculumCallback, self).__init__(verbose)
        
        # 🌟 在這裡設定各關卡的「通關平均分數」門檻
        # 你可以根據實際訓練情況調整這些數字
        self.stage_thresholds = {
            1: 800.0,   # 第一關平均拿到 800 分才准晉級
            2: 850.0,   # 第二關平均 850 分晉級
            3: 900.0,
            4: 950.0
        }

    def _on_step(self) -> bool:
        # 確保模型已經跑完足夠的回合來計算平均值
        if len(self.model.ep_info_buffer) >= 10:  
            # 計算最近 100 局 (或 buffer 內所有局) 的平均分數，這就是 rollout/ep_rew_mean
            ep_rew_mean = np.mean([ep_info["r"] for ep_info in self.model.ep_info_buffer])
            
            # 取得目前的關卡 (透過 get_attr 穿透 SB3 的 VecEnv 封裝)
            current_stage = self.training_env.get_attr('current_stage')[0]

            if current_stage in self.stage_thresholds:
                threshold = self.stage_thresholds[current_stage]
                
                # 如果分數達標！
                if ep_rew_mean >= threshold:
                    new_stage = current_stage + 1
                    
                    # 遠端遙控環境，更新關卡
                    self.training_env.env_method('set_current_stage', new_stage)
                    
                    print(f"\n" + "="*50)
                    print(f"🎉🎉🎉 平均分數達標！({ep_rew_mean:.2f} >= {threshold}) 🎉🎉🎉")
                    print(f"🚀🚀🚀 全面晉級至第 {new_stage} 關！ 🚀🚀🚀")
                    print("="*50 + "\n")
                    
                    # 🌟 關鍵操作：晉級後清空分數暫存！
                    # 避免上一關的高分「污染」下一關的平均計算
                    self.model.ep_info_buffer.clear()
                    
        return True


def main():
    # 1. 初始化 ROS 2
    rclpy.init()
    
    # 2. 實例化環境與節點
    env = ShipGimbalEnv()
    
    # 3. 設定多執行緒：這非常重要！
    # 讓 ROS 2 的訂閱者(Subscriber)在背景不斷更新感測器資料
    # 才不會被 PPO 演算法卡死
    executor = MultiThreadedExecutor()
    executor.add_node(env)
    
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # 宣告 model 變數為 None，確保在 exception 區塊可以存取到它
    model = None

    try:
        # 4. 建立與設定 PPO 模型
        print("🤖 初始化 PPO 模型...")
        model = PPO(
            "MlpPolicy", 
            env, 
            verbose=1, 
            n_steps=4096,           # 🌟 核心修改：從 2048 提高到 4096！大腦會收集兩倍的資料才進行更新
            batch_size=512,         # 🌟 核心修改：配合 n_steps 提高到 512，讓每次梯度下降更穩定
            learning_rate=0.0003,   
            ent_coef=0.05,          # 🌟 核心修改：提高探索權重 (0.01 -> 0.02)，逼它開局更積極甩頭
            # ==========================================
            # 🌟 新增的進階狙擊手參數
            # ==========================================
            gamma=0.995,            # 🌟 折扣因子：提高到 0.995，讓它更重視未來開火命中的巨大紅利
            policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256])), # 🌟 擴充腦容量：加深神經網路，讓它有能力理解複雜的重力井
            # ==========================================
            tensorboard_log="./ppo_gimbal_tensorboard/",
            device="cuda"            
        )
        
        # 5. 開始訓練
        print("🚀 開始訓練！請確保 Gazebo 已開啟且 Topic 正常運作...")
        # 這裡設定跑 100,000 步，可依需求增減
        #model.learn(total_timesteps=1000000) # 可以直接把上限拉高，反正規可以手動停

        # 建立 Callback
        curriculum_callback = CurriculumCallback()

        # 在 learn 裡面加上 callback 參數
        model.learn(total_timesteps=3000000, callback=curriculum_callback)
            
        # 6. 訓練自然完成，儲存模型
        model.save("ship_gimbal_ppo_model_final")
        print("💾 訓練完成，模型儲存成功：ship_gimbal_ppo_model_final.zip")

    except KeyboardInterrupt:
        # 🌟 修改 2：捕捉到 Ctrl+C 中斷時，立刻將目前的模型存檔！
        print("\n🛑 偵測到使用者手動中斷 (Ctrl+C)！正在儲存當前訓練進度...")
        if model is not None:
            model.save("ship_gimbal_ppo_model_interrupted")
            print("💾 提前中斷模型儲存成功：ship_gimbal_ppo_model_interrupted.zip")
            
    finally:
        # 安全關閉與清理資源
        env.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()