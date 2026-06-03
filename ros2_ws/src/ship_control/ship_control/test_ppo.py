import rclpy
from stable_baselines3 import PPO

# 引入我們辛苦打造的環境
from gimbal_env import ShipGimbalEnv

def main():
    # 1. 初始化 ROS 2
    rclpy.init()
    
    # 2. 實例化環境與節點
    env = ShipGimbalEnv()
    
    # 🛑 (已刪除多執行緒 Executor，交給 env.step 內建的 spin_once 處理)

    MODEL_PATH = "ship_gimbal_ppo_model_final"

    try:
        # 3. 載入大腦
        print(f"🧠 正在載入訓練好的 AI 大腦：{MODEL_PATH} ...")
        model = PPO.load(MODEL_PATH, env=env)
        print("✅ 大腦載入成功！開始執行追蹤任務...")
        
        # 4. 重置環境，取得開局第一眼看到的畫面 (Observation)
        obs, info = env.reset()
        
        # 5. 進入無限迴圈
        while True:
            # deterministic=True：進入考場模式，拔除隨機探索，發揮最強實力
            action, _states = model.predict(obs, deterministic=True)
            
            # 將動作傳給環境，雲台馬達轉動，並回傳新的畫面與分數
            obs, reward, terminated, truncated, info = env.step(action)
            
            # 如果回合結束 (例如擊中目標或追丟超時)
            if terminated or truncated:
                print("🔄 回合結束，重新重置世界尋找目標...")
                obs, info = env.reset()
                
            # 🛑 (已刪除 time.sleep，讓 AI 以訓練時的原生速度全速運作！)

    except FileNotFoundError:
        print(f"❌ 找不到模型檔案：{MODEL_PATH}.zip，請確認檔名是否正確！")
    except KeyboardInterrupt:
        print("\n🛑 測試結束，準備關閉程式。")
    finally:
        # 安全關閉
        env.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()