"""
CAMERA LAUNCH FILE
Package : ship_bringup
Launches : Gazebo world + image bridge + YOLO + pixel_to_angle + Foxglove
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Path to world file via ROS2 package share
    world_file = os.path.join(
        get_package_share_directory('ship_simulation'),
        'worlds',
        'camera_world.sdf'
    )
    
    pkg_sim_share = get_package_share_directory('ship_simulation')
    models_path = os.path.join(pkg_sim_share, 'models')

    return LaunchDescription([

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', 
                               value=models_path),
        
        # ------- 1. Gazebo -------
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # ------- 2. Foxglove bridge -------
        TimerAction(period=4.0, actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                parameters=[{'port': 8765}],
                output='screen'
            ),
        ]),

        # ------- 3. ROS-Gazebo image bridge -------
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=['/camera/image_raw'],
                output='screen'
            ),
        ]),

        # ------- 4. YOLO detection node -------
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_vision',
                executable='yolo_detection_node',
                output='screen'
            ),
        ]),

        # ------- 5. Pixel to angle node -------
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_vision',
                executable='pixel_to_angle_node',
                output='screen'
            ),
        ]),

    ])