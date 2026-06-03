Here is the English translation of your README file:

# Ship Gimbal Tracking Project

This project aims to train a ship-mounted gimbal for target tracking and prediction using Reinforcement Learning (PPO). The development environment is based on **Ubuntu 24.04**, **ROS 2 Jazzy**, and **Gazebo Harmonic**.

## 1. Prerequisites
Ensure the following core frameworks are correctly installed on your system:
*   **ROS 2 Jazzy Jalisco:** [Official Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
*   **Gazebo Harmonic:** [Official Installation Guide](https://gazebosim.org/docs/harmonic/install)

**💡 Environment Verification:**
```bash
echo $ROS_DISTRO        # Expected output: jazzy
gz sim --version        # Should display Harmonic (Sim 9) version information
```

## 2. Folder Structure
```text
ship_gimbal_tracking/
├── ros2_ws/                 # ROS 2 Workspace
│   ├── src/
│   │   ├── ship_bringup/    # Launchers
│   │   ├── ship_control/    # RL training logic and environment
│   │   ├── ship_vision/     # Image processing (YOLO/OpenCV/Debug)
│   │   ├── ship_description/# URDF and model definitions
│   │   ├── ship_msgs/       # ROS message definitions
│   │   └── ship_simulation/ # Gazebo worlds and plugins
├── README_CH.md             # README (Traditional Chinese version)
└── README.md
```

## 3. Setup

### 3.1.1 Create `ship_gimbal` Virtual Environment : for ROS 2
Execute the following commands in the project root directory to create and activate the environment:
```bash
cd ~/ship/ship_gimbal_tracking/ # Adjust the path to your actual project directory
python3 -m venv ship_gimbal
source ship_gimbal/bin/activate
```

### 3.1.2 Install System Dependencies for `ship_gimbal` : for ROS 2
Ensure ROS 2 Jazzy is installed. Then, install the necessary core Python dependencies:
```bash
pip install "numpy<2" matplotlib opencv-python ultralytics foxglove-sdk
# For GPU computing, install the appropriate Torch version based on your hardware architecture
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm7.2
```

### 3.2.1 Create `rl_env` Virtual Environment : for PPO
Execute the following commands in the project root directory to create and activate the environment:
```bash
cd ~/ship/ship_gimbal_tracking/ # Adjust the path to your actual project directory
python3 -m venv rl_env
source rl_env/bin/activate
```

### 3.2.2 Install System Dependencies for `rl_env` : for PPO
Ensure ROS 2 Jazzy is installed. Then, install the necessary core Python dependencies:
```bash
pip install stable-baselines3[extra] gymnasium numpy pyyaml setuptools
# For GPU computing, install the appropriate Torch version based on your hardware architecture
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm7.2
```

## 4. Build & Workflow

### 4.1 Workspace Management
If you encounter errors, clean and rebuild the workspace:
```bash
cd ~/ship/ship_gimbal_tracking/ros2_ws # Adjust the path to your actual project directory
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. RL Workflow
Open three separate terminals and proceed as follows:

### Terminal 1: Start Simulation
This window loads the Gazebo world and initializes the system plugins.
```bash
cd ~/ship/ship_gimbal_tracking/ # Adjust path
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Set environment variables
export PROJECT_ROOT=$PWD
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PROJECT_ROOT/ros2_ws/install/lib:$PROJECT_ROOT/ros2_ws/install/gazebo_maritime/lib
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:$PROJECT_ROOT/ros2_ws/install/lib:/$PROJECT_ROOT/ros2_ws/install/gazebo_maritime/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PROJECT_ROOT/ros2_ws/install/lib:$PROJECT_ROOT/ros2_ws/install/gazebo_maritime/lib:$PROJECT_ROOT/ros2_ws/src/ship_simulation/external/gazebo_maritime_ws/src/gazebo_maritime/lib
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

# Build and Launch
cd ros2_ws
colcon build --symlink-install --packages-select ship_vision
ros2 launch ship_bringup gimbal_launch.py
```

### Terminal 2: Monitor Sensors
Used to verify the target contact status.
```bash
cd ~/ship/ship_gimbal_tracking/ # Adjust path
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
gz topic -e -t /world/gimbal_world/model/wamv/link/person_link/sensor/person_contact/contact
```

### Terminal 3: Execute Training
Start the PPO training process.
```bash
cd ~/ship/ship_gimbal_tracking/ # Adjust path
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source rl_env/bin/activate
python3 ros2_ws/src/ship_control/ship_control/train_ppo.py
```

---
*Note: Parameters are currently hardcoded in the `ship_control` module. To modify the reward function or training hyperparameters, please edit the corresponding Python scripts directly.*