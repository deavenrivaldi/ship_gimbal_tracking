# Ship Gimbal Tracking Project

本專案旨在透過強化學習（PPO）訓練船舶雲台進行目標追蹤與預測。開發環境基於 **Ubuntu 24.04**、**ROS 2 Jazzy** 與 **Gazebo Harmonic**。

## 1. 環境前提 (Prerequisites)
確保系統已正確安裝以下核心框架：
*   **ROS 2 Jazzy Jalisco:** [官方安裝指南](https://docs.ros.org/en/jazzy/Installation.html)
*   **Gazebo Harmonic:** [官方安裝指南](https://gazebosim.org/docs/harmonic/install)

**💡 環境驗證指令：**
```bash
echo $ROS_DISTRO        # 應輸出: jazzy
gz sim --version        # 應顯示 Harmonic (Sim 9) 版本資訊
```

## 2. 目錄結構 (Folder Structure)
```text
ship_gimbal_tracking/
├── ros2_ws/                 # ROS 2 工作區
│   ├── src/
│   │   ├── ship_bringup/    # 啟動設定 (Launchers)
│   │   ├── ship_control/    # RL 訓練邏輯與環境
│   │   ├── ship_vision/     # 影像處理 (YOLO/OpenCV/Debug)
│   │   ├── ship_description/# URDF 與模型定義
│   │   ├── ship_msgs/       # ROS 訊息定義
│   │   └── ship_simulation/ # Gazebo 世界與插件
├── README_CH.md             # 繁體中文版 README
└── README.md
```

## 3. 環境準備 (Setup)

### 3.1.1 建立 ship_gimbal 虛擬環境 : for ros2
請在專案根目錄下執行以下指令建立並啟用環境：
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
python3 -m venv ship_gimbal
source ship_gimbal/bin/activate
```

### 3.1.2 ship_gimbal 系統相依安裝 : for ros2
確保系統已安裝 ROS 2 Jazzy。並安裝必要的 Python 核心相依套件：
```bash
pip install "numpy<2" matplotlib opencv-python ultralytics foxglove-sdk
# 若使用 GPU 運算，請依照硬體架構安裝對應的 Torch 版本
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm7.2
```

### 3.2.1 建立 rl_env 虛擬環境 : for ppo
請在專案根目錄下執行以下指令建立並啟用環境：
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
python3 -m venv rl_env
source rl_env/bin/activate
```

### 3.2.2 rl_env 系統相依安裝 : for ppo
確保系統已安裝 ROS 2 Jazzy。並安裝必要的 Python 核心相依套件：
```bash
pip install stable-baselines3[extra] gymnasium numpy pyyaml setuptools
# 若使用 GPU 運算，請依照硬體架構安裝對應的 Torch 版本
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm7.2
```

## 4. 編譯與執行 (Build & Workflow)

### 4.1 工作區管理
若專案異常，請清理後重編：
```bash
cd ~/ship/ship_gimbal_tracking/ros2_ws # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. 強化學習訓練流程 (RL Workflow)
請依序開啟三個終端機：

### 終端機 1：啟動模擬環境
此視窗負責載入 Gazebo 世界並啟動系統插件。
```bash
cd ~/ship/ship_gimbal_tracking/ # 請依實際路徑調整 : 專案 ship_gimbal_tracking 資料夾路徑
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

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

---
*註：參數配置目前硬編碼於 `ship_control` 模組中。如需修改獎勵函數或訓練參數，請直接編輯對應 Python 腳本。*