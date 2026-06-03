import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from stable_baselines3 import PPO

# 引入剛剛建立好的環境類別
from gimbal_env import ShipGimbalEnv

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
        model.learn(total_timesteps=2000000) # 可以直接把上限拉高，反正規可以手動停
        
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