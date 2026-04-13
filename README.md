## Github Workflow

```
git pull # work on ROS2 packages
git add
git commit -m "update message"
git push
```

## Folder Structure & Functions

```
ship_gimbal_tracking
ros2_ws
            src
                        ship_bringup (pkg) <- launcher
                                    ship_bringup
                                                camera_launch.py
                                                gimbal_launch.py
                        ship_control (pkg) <- control logic
                        ship_vision (pkg) <- openCV / detection, camera processing
                                    ship_vision
                                    yolo_detection_node.py
                                    pixel_to_angle_node.py
                                    plot_debug_node.py
                                    fg_plot_debug_node.py
                        ship_description (pkg) <- URDF / robot model, sensors / joints
                        ship_simulation (pkg) <- gz world, models, plugins
                                    worlds
                                                camera_world.sdf
                                                gimbal_world.sdf
                                    models
                                    external

            build
            install
            log
ship_gimbal     <- python venv folder
.gitignore
README.md
```

### Create New ROS Package

Inside the ros2_ws/src folder, run in the terminal

```
ros2 pkg create --build-type ament_python $package_name
```

> change $package_name with the name of package you want to create (e.g., ship_bringup)

## Python Framework

**python virtual environment:** ship_gimbal  
create new venv

```
cd ~/projects/ship_gimbal_tracking
python3 -m venv ship_gimbal
```

activate venv before launching ROS 2 by running in terminal

```
cd ~/projects/ship_gimbal_tracking
source ship_gimbal/bin/activate
```

### Libraries

```
numpy               => pip install "numpy<2"
matplotlib          => pip install matplotlib
torch, torchvision  => pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm7.2
opencv              => pip install opencv-python
yolo                => pip install -U ultralytics
```

# ROS 2 Framework

### Reset Colcon

If the project breaks, reset colcon and build it again

```
cd ~/projects/ship_gimbal_tracking/ros2_ws
rm -rf build install log
```

### Build Workspace

after every update, rebuild workspace by running in terminal

```
cd ~/projects/ship_gimbal_tracking/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
colcon build --symlink-install --packages-select ship_vision
source install/setup.bash
```

## World Launcher:

### Launch with NVIDIA CUDA

after building workspace run

```
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
ros2 launch ship_bringup $XXX_launch.py
```

or

### Launch with AMD GPU

```
export HSA_OVERRIDE_GFX_VERSION=10.3.0
ros2 launch ship_bringup $XXX_launch.py
```

> change $XXX with the world that you want to launch {camera, gimbal}

## Debugging

### Target Position Debug Plot (vision node):

Open a new terminal and run

```
cd ~/projects/ship_gimbal_tracking/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ship_vision plot_debug_node
```

### Foxglove Dashhboard (vision node):

In Foxglove open a new image window and change the topic to

```
/debug/image
```

### Nodes and Topics Verification:

Open a new terminal and run

```
cd ~/projects/ship_gimbal_tracking/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 node list
ros2 topic list
```

#### Expected Nodes:

    /yolo_detection_node
    /pixel_to_angle_node
    /foxglove_bridge

#### Expected Topics:

    /camera/image_raw
    /target/pixel_center
    /gimbal/angle_command
    /debug/image

### Verify IMU publishing:

ros2 topic echo /imu/data --once
ros2 topic echo /debug/imu_angles --once
ros2 topic echo /gimbal/roll_correction --once

#### Expected Result (on flat ground):

    /debug/imu_angles:   x≈0.0  y≈0.0  z≈0.0   (flat)
    /gimbal/roll_correction:  x≈0.0  y≈0.0      (no correction needed)
