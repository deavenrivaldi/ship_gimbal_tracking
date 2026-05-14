import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_projectile_gz = get_package_share_directory('projectile_gz')
    pkg_projectile_bringup = get_package_share_directory('projectile_bringup')

    params_file = os.path.join(
        pkg_projectile_bringup,
        'config',
        'projectile_params.yaml'
    )

    with open(params_file, 'r') as f:
        cfg = yaml.safe_load(f)

    ros_params = cfg['gz_interface']['ros__parameters']
    world_name = ros_params['world_name']
    world_file = ros_params['world_file']

    world_path = os.path.join(pkg_projectile_gz, 'models', world_file)

    return LaunchDescription([

        # 1. Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        # 2. Bridge spawn
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean'
            ],
            output='screen'
        ),

        # 3. Bridge wrench
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/{world_name}/wrench@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench'
            ],
            output='screen'
        ),

        # 4. gz_interface node
        Node(
            package='projectile_gz',
            executable='gz_interface',
            name='gz_interface',
            output='screen',
            parameters=[params_file]
        ),
    ])