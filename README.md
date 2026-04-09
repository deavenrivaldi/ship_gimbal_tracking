workflow with github:
git pull # work on ROS2 packages
git add .
git commit -m "update message"
git push

create ros package: ros2 pkg create --build-type ament_python $package_name

folder structure & function:
ship_gimbal_tracking
ros2_ws
src
ship_bringup (pkg) <- launcher
ship_bringup
camera_launch.py
ship_control (pkg) <- control logic
ship_vision (pkg) <- openCV / detection, camera processing
ship_vision
yolo_detection_node.py
pixel_to_angle_node.py
ship_description (pkg) <- URDF / robot model, sensors / joints
ship_simulation (pkg) <- gz world, models, plugins
worlds
camera_world.sdf
models
external

            build
            install
            log
        ship_gimbal     <- python venv
        .gitignore
        README.md

python virtual environment: ship_gimbal
activate venv: source ship_gimbal/bin/activate

python libraries:

build workspace
cd ~/projects/ship_gimbal_tracking/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
colcon build --symlink-install --packages-select ship_vision
source install/setup.bash

launcher:

    camera_world:
        export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
        ros2 launch ship_bringup camera_launch.py

    debut_plot:
        ros2 run ship_vision plot_debug_node

verification:
source /opt/ros/jazzy/setup.bash
source ~/ship_gimbal_tracking/ros2_ws/install/setup.bashInitial Commit

test push from zen-ubuntu 24.04

    ros2 node list
    ros2 topic list

    expected nodes:
        /yolo_detection_node
        /pixel_to_angle_node
        /foxglove_bridge
    expected topics:
        /camera/image_raw
        /target/pixel_center
        /gimbal/angle_command
        /debug/image
