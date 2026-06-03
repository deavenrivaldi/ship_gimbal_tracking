### 一、 訓練環境與參數設定
*   **視覺與運算：**
    *   鏡頭更新率提升至 120Hz。
    *   環境重啟機制：確保主船與目標同步刷新，避免主船漂離；雲台不刷新以維持鏡頭狀態。
    *   優化運算：先以 Gazebo 畫面確認獎勵合理性與收斂情況，後續訓練改用 `-s` 模式（無畫面）並調用最適合的 CPU 或 GPU 資源。
*   **雲台運動限制：**
    *   移除 IMU 與 Pixel-to-Angle 的限制。
    *   放寬雲台轉動範圍，實現 360 度無限旋轉，並提升轉動速度。
*   **訓練階段策略（課程學習）：**
    1. 第一關：目標搜尋。
    2. 第二關：適應位置變換。
    3. 第三關：熟悉旋轉。
    4. 第四關：目標跟隨與預測。
*   **生成與晉級機制：**
    *   目標生成邏輯：根據鏡頭當前朝向生成，防止因生成在眼前造成的「幸運擊中」；確保 AI 具備主動搜尋能力。
    *   晉級判定：採用平均回報（`rollout/ep_rew_mean`）作為晉級門檻，確保模型完全收斂後再進入下一關。

### 二、 獎勵函數設計
*   **階梯式追蹤獎勵：**
    *   誤差 < 2°：+60.0 分（完美爆頭）。
    *   誤差 < 8°：+20.0 分（穩定追蹤）。
    *   誤差 < 20°：+5.0 分（邊緣捕捉）。
    *   視野外：無獎勵。
    *   獎勵扣除：根據動作幅度 (`action_magnitude`) 進行負回饋。
*   **平滑度與控制懲罰：**
    *   防抖動懲罰：針對來回快速變換的動作施加懲罰，避免雲台頻繁抖動（調整此參數需考量對旋轉追蹤的負面影響）。
    *   Pitch 水平懲罰：`reward -= (action_pitch**2) * 0.05`，鼓勵保持水平掃描。
*   **時效控制：**
    *   訓練時間設定：設定為約繞行一圈的時間即刷新世界，避免 AI 陷入嘗試尋找 Bug 而非完成目標。

### 三、 執行與部署指令
*   **環境準備：**
    *   安裝依賴：`pip install stable-baselines3[extra] gymnasium numpy`
*   **終端機操作：**
    *   **First Terminal (監控)：** 監聽接觸感測器。
        ```bash
        source /opt/ros/jazzy/setup.bash
        . ~/workspace/install/setup.bash
        gz topic -e -t /world/gimbal_world/model/wamv/link/person_link/sensor/person_contact/contact
        ```
    *   **Second Terminal (訓練)：**
        ```bash
        source /opt/ros/jazzy/setup.bash
        source ~/ship/ship_gimbal_tracking/ros2_ws/install/setup.bash
        source ~/ship/ship_gimbal_tracking/ros2_ws/src/ship_control/ship_control/rl_env/bin/activate
        cd ~/ship/ship_gimbal_tracking/ros2_ws/src/ship_control/ship_control/
        python3 train_ppo.py
        ```
    *   **Third Terminal (啟動環境)：**
        ```bash
        # 設定路徑環境變數
        export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/home/wuru/ship/ship_gimbal_tracking/ros2_ws/install/lib:/home/wuru/ship/ship_gimbal_tracking/ros2_ws/install/gazebo_maritime/lib
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/wuru/ship/ship_gimbal_tracking/ros2_ws/install/lib:/home/wuru/ship/ship_gimbal_tracking/ros2_ws/src/ship_simulation/external/gazebo_maritime_ws/src/gazebo_maritime/lib
        export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
        # 編譯與執行
        colcon build --symlink-install --packages-select ship_vision
        ros2 launch ship_bringup gimbal_launch.py
        ```

### 四、 強化學習核心機制說明
*   **信用分配 (Credit Assignment)：** 利用 Actor-Critic 架構，透過折現因子 ($\gamma$) 將延遲的獎勵訊號回推至關鍵決策時間點。
*   **狀態空間優化：** 必須納入目標速度（Velocity）資訊，協助評論家（Critic）區分命中與未命中情境，進而建立對提前量的預判能力。