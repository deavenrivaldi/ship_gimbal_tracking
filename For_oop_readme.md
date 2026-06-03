# Project README: Ship Gimbal Tracking

本專案旨在透過強化學習（PPO）訓練船舶雲台進行目標追蹤，包含模擬環境架設、視覺辨識與訓練流程。

## 1. 環境準備 (Setup)

### 1.1.1 建立虛擬環境 - ship_gimbal : for ros2
請在專案根目錄下執行以下指令建立並啟用環境：
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
python3 -m venv ship_gimbal
source ship_gimbal/bin/activate
```

### 1.1.2 系統相依安裝 - ship_gimbal : for ros2
確保系統已安裝 ROS 2 Jazzy。並安裝必要的 Python 核心相依套件：
```bash
pip install "numpy<2" matplotlib opencv-python ultralytics foxglove-sdk
# 若使用 GPU 運算，請依照硬體架構安裝對應的 Torch 版本
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm7.2
```

### 1.2.1 建立虛擬環境 - rl_env : for ppo
請在專案根目錄下執行以下指令建立並啟用環境：
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
python3 -m venv rl_env
source rl_env/bin/activate
```

### 1.2.2 系統相依安裝 - rl_env : for ppo
確保系統已安裝 ROS 2 Jazzy。並安裝必要的 Python 核心相依套件：
```bash
pip install stable-baselines3[extra] gymnasium numpy pyyaml setuptools
# 若使用 GPU 運算，請依照硬體架構安裝對應的 Torch 版本
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm7.2
```

## 2. 設定說明 (Configuration)
目前所有參數（如獎勵權重、雲台轉動限制、學習率等）均已硬編碼於訓練腳本中。如需修改行為，請直接編輯對應的 Python 源碼檔案。

## 3. 執行流程 (Usage)

請依序開啟三個終端機進行初始化與訓練：

### 終端機 1：啟動模擬環境
此視窗負責載入 Gazebo 世界並啟動系統插件。
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash # (project/ship_test)需改成本地路徑

# 設定環境變數
export PROJECT_ROOT=$PWD
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PROJECT_ROOT/ros2_ws/install/lib:$PROJECT_ROOT/ros2_ws/install/gazebo_maritime/lib
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:$PROJECT_ROOT/ros2_ws/install/lib:/$PROJECT_ROOT/ros2_ws/install/gazebo_maritime/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PROJECT_ROOT/ros2_ws/install/lib:$PROJECT_ROOT/ros2_ws/install/gazebo_maritime/lib:$PROJECT_ROOT/ros2_ws/src/ship_simulation/external/gazebo_maritime_ws/src/gazebo_maritime/lib
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

# 編譯與啟動
cd ros2_ws
colcon build --symlink-install --packages-select ship_vision
ros2 launch ship_bringup gimbal_launch.py
```

### 終端機 2：監控感測器
用於確認目標接觸狀態。
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
gz topic -e -t /world/gimbal_world/model/wamv/link/person_link/sensor/person_contact/contact
```

### 終端機 3：執行訓練
啟動 PPO 訓練程序。
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source rl_env/bin/activate
python3 ros2_ws/src/ship_control/ship_control/train_ppo.py
```

## 4. 訓練策略說明
*   **階段式學習：** 訓練分為「尋找目標」、「位置變換」、「旋轉適應」、「跟隨預測」四階段。
*   **晉級機制：** 系統依據 `rollout/ep_rew_mean`（平均回報）作為收斂基準，達到設定閾值後自動晉級下一階段。
*   **重置邏輯：** 為防止過度擬合與僥倖擊中，世界刷新時目標生成位置將依據鏡頭當前朝向調整。