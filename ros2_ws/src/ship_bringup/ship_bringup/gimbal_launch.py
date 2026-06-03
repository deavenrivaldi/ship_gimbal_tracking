"""
GIMBAL LAUNCH FILE
Package : ship_bringup
Launches : Gazebo gimbal world + image bridge + vision nodes + gimbal controller + Foxglove
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # gimbal bridge 
     # 定義 Bridge Node
    pkg_ship_bringup = get_package_share_directory('ship_bringup')
    bridge_file = os.path.join(
        pkg_ship_bringup,
        'config',
        'ship_bridge.yaml'
    )

    with open(bridge_file, 'r') as f:
        cfg = yaml.safe_load(f)

    ros_params = cfg['gz_interface']['ros__parameters']
    world_name = ros_params['world_name']
    world_file = ros_params['world_file']

    pkg_sim_share = get_package_share_directory('ship_simulation')
    world_file_path = os.path.join(pkg_sim_share, 'worlds', world_file)
    models_path = os.path.join(pkg_sim_share, 'models')

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gimbal_bridge',
        output='screen',
        arguments=[
            '/gimbal/joint_trajectory@trajectory_msgs/msg/JointTrajectory@gz.msgs.JointTrajectory',
            f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean',
            f'/world/{world_name}/wrench@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench',
            
            
            f'/model/gimbal_wam_v/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',

            # 🌟 拔除 wam-v 的連字號，並確保使用的是 person_link
            '/world/gimbal_world/model/wamv/link/person_link/sensor/person_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        
        ],

        
        remappings=[
        ('/model/gimbal_wam_v/pose', '/tf')
    ],
    
    
    )

    # add plgin search path : ship_gimbal_tracking/ros2_ws/src/ship_simulation/external/gazebo_maritime_ws/src/gazebo_maritime/lib
    current_file_dir = os.path.dirname(os.path.realpath(__file__))
    plugin_path = os.path.abspath(os.path.join(current_file_dir, '../../../../../src/ship_simulation/external/gazebo_maritime_ws/src/gazebo_maritime/lib'))
    # corrention to plugin path
    if not os.path.exists(plugin_path):
        print(f"警告：找不到插件路徑 {plugin_path}")

    pkg_vis_share = get_package_share_directory('ship_vision')
    project_root = os.path.abspath(os.path.join(pkg_vis_share, '../../../../..'))

    venv_python = os.path.join(project_root, 'ship_gimbal', 'bin', 'python3')  

    return LaunchDescription([
        bridge_node,
        
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', 
                               value=models_path), 
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', 
                               value=plugin_path),     

        # ------- 1. Gazebo -------
        ExecuteProcess(
            cmd=['gz', 'sim','-r', world_file_path],
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
        

        # ------- 4. ROS-Gazebo IMU bridge -------
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'
                ],
                output='screen'
            ),
        ]),
        
        # ------- 5. YOLO detection node -------
        #TimerAction(period=7.0, actions=[
        #    Node(
        #        package='ship_vision',
        #        executable='yolo_detection_node',
        #        output='screen'
        #    ),
        #]),
        
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_vision',
                executable=venv_python,
                arguments=[os.path.join(pkg_vis_share, 'yolo_detection_node.py')],
                output='screen'
            ),
        ]),

        # ------- 6. Pixel to angle node -------
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_vision',
                executable='pixel_to_angle_node',
                output='screen'
            ),
        ]),
        
        # ------- 7. Plot debug node -------
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_vision',
                executable='fg_plot_debug_node',
                output='screen'
            ),
        ]),
        
         #------- 8. IMU stabilizer node -------
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_control',
                executable='imu_stabilizer_node',
                output='screen'
            ),
        ]),

        # ------- 9. Gimbal controller node -------
  #      TimerAction(period=8.0, actions=[
    #        Node(
    #            package='ship_control',
  #             executable='gimbal_controller_node',
     #           parameters=[bridge_file],
      #          output='screen'
      #      ),
       # ]),
        


         # ------- 9. Fire node -------
        TimerAction(period=8.0, actions=[
            Node(
                package='ship_control',
                executable='fire_node',
                parameters=[bridge_file],
                output='screen'
            ),
        ]),

        
    ])
