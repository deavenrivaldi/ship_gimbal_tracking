#!/bin/bash
# 1. 舊依賴
export HSA_OVERRIDE_GFX_VERSION=10.3.0 
# 2. 加入 models depend 
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ship_gimbal_tracking/ship_gimbal_tracking/ros2_ws/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ethan/ship_gimbal_tracking/ship_gimbal_tracking/ros2_ws/src
# 3. 載入 ROS 2 與工作空間環境 (關鍵！)
source /opt/ros/jazzy/setup.bash
source /home/ethan/ship_gimbal_tracking/ship_gimbal_tracking/ros2_ws/install/setup.bash

echo "環境已載入：ship_gimbal_tracking workspace"