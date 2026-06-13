"""
GIMBAL STABILIZATION LAUNCH FILE
Package : ship_bringup

Plugin loading strategy:
  GZ_SIM_SYSTEM_PLUGIN_PATH, IGN_GAZEBO_SYSTEM_PLUGIN_PATH, and LD_LIBRARY_PATH
  are expected to already be set correctly in ~/.bashrc (they worked before).
  This launch file reads those values and passes them through unchanged to gz sim
  via additional_env — it never overwrites them with guessed paths.

  The only var this file sets itself is GZ_SIM_RESOURCE_PATH, which must point
  to the installed models directory and changes with each colcon build.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package paths ────────────────────────────────────────────────────
    pkg_bringup = get_package_share_directory('ship_bringup')
    pkg_sim     = get_package_share_directory('ship_simulation')
    pkg_vis     = get_package_share_directory('ship_vision')

    bridge_file     = os.path.join(pkg_bringup, 'config', 'ship_bridge.yaml')
    with open(bridge_file, 'r') as f:
        cfg = yaml.safe_load(f)
    ros_params      = cfg['gz_interface']['ros__parameters']
    world_name      = ros_params['world_name']
    world_file      = ros_params['world_file']
    world_file_path = os.path.join(pkg_sim, 'worlds', world_file)
    models_path     = os.path.join(pkg_sim, 'models')

    # ── Plugin paths: trust ~/.bashrc, pass through unchanged ────────────
    # Your bashrc already sets these correctly (the world worked before).
    # We read the current shell values and forward them into gz sim's process
    # via additional_env so they are guaranteed to be present at dlopen time.
    gz_plugin_path  = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH',    '')
    ign_plugin_path = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    ld_path         = os.environ.get('LD_LIBRARY_PATH',               '')

    # GZ_SIM_RESOURCE_PATH is the one path we must set ourselves — it
    # points to the installed models dir which changes after each build.
    # Append to whatever bashrc already set (don't overwrite).
    existing_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = ':'.join(p for p in [models_path, existing_resource] if p)

    # Print what we resolved so you can verify at launch time
    print('\n[gimbal_launch] ── Environment forwarded to gz sim ─────────')
    print(f'  GZ_SIM_RESOURCE_PATH      : {resource_path}')
    print(f'  GZ_SIM_SYSTEM_PLUGIN_PATH : {gz_plugin_path or "(from bashrc — not overridden)"}')
    print(f'  LD_LIBRARY_PATH           : {"(set)" if ld_path else "(empty — check bashrc)"}')
    print('─────────────────────────────────────────────────────────────\n')

    # Passed directly into gz sim's subprocess — this is what actually works.
    gz_env = {
        'GZ_SIM_RESOURCE_PATH':          resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH':     gz_plugin_path,
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ign_plugin_path,
        'LD_LIBRARY_PATH':               ld_path,
    }

    # ── Virtual-env python for YOLO ──────────────────────────────────────
    project_root = os.path.abspath(os.path.join(pkg_sim, '../../../../..'))
    venv_python  = os.path.join(project_root, 'ship_gimbal', 'bin', 'python3')

    # ── Main ROS↔Gz bridge ───────────────────────────────────────────────
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gimbal_bridge',
        output='screen',
        arguments=[
            f'/gimbal/joint_trajectory@trajectory_msgs/msg/JointTrajectory@gz.msgs.JointTrajectory',
            f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean',
            f'/world/{world_name}/wrench@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench',
            f'/model/gimbal_wam_v/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            f'/world/{world_name}/model/wamv/link/person_link/sensor/person_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[('/model/gimbal_wam_v/pose', '/tf')],
    )

    return LaunchDescription([

        # Only set resource path here — plugin/library paths come from
        # ~/.bashrc unchanged, so we never overwrite them in the launch file.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),

        bridge_node,

        # ── 1. Gazebo ─────────────────────────────────────────────────────
        # additional_env forwards the current shell env (including bashrc
        # plugin paths) directly into gz sim's subprocess at spawn time.
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file_path],
            additional_env=gz_env,
            output='screen'
        ),

        # ── 2. Foxglove bridge ────────────────────────────────────────────
        TimerAction(period=4.0, actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                parameters=[{'port': 8765}],
                output='screen'
            ),
        ]),

        # ── 3. Camera image bridge ────────────────────────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=['/camera/image_raw'],
                output='screen'
            ),
        ]),

        # ── 4. Boat IMU bridge ────────────────────────────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='boat_imu_bridge',
                arguments=['/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU'],
                output='screen'
            ),
        ]),

        # ── 5. Camera IMU bridge ──────────────────────────────────────────
        # Bridges /camera_imu/data_raw (Gz topic set in sensor SDF)
        # and remaps it to /camera_imu/data for our nodes.
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='camera_imu_bridge',
                arguments=['/camera_imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                remappings=[('/camera_imu/data_raw', '/camera_imu/data')],
                output='screen'
            ),
        ]),

        # ── 6. Joint-state bridge ─────────────────────────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='joint_state_bridge',
                arguments=[
                    f'/world/{world_name}/model/gimbal/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
                ],
                output='screen'
            ),
        ]),

        # ── 7. Gimbal PID stabilizer ──────────────────────────────────────
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_control',
                executable='gimbal_stabilizer_node',
                name='gimbal_stabilizer_node',
                output='screen'
            ),
        ]),

        # ── 8. YOLO detection (runs inside ship_gimbal venv) ──────────────
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_vision',
                executable=venv_python,
                arguments=[os.path.join(pkg_vis, 'yolo_detection_node.py')],
                output='screen'
            ),
        ]),

        # ── 9. Pixel-to-angle ─────────────────────────────────────────────
        TimerAction(period=7.0, actions=[
            Node(
                package='ship_vision',
                executable='pixel_to_angle_node',
                output='screen'
            ),
        ]),

        # ── 10. IMU comparison dashboard ──────────────────────────────────
        # Subscribe to /debug/imu_comparison in Foxglove as an Image panel.
        TimerAction(period=8.0, actions=[
            Node(
                package='ship_vision',
                executable='imu_dashboard_node',
                name='imu_dashboard_node',
                output='screen'
            ),
        ]),

        # ── 11. Roll/YOLO debug plot ──────────────────────────────────────
        TimerAction(period=8.0, actions=[
            Node(
                package='ship_vision',
                executable='fg_plot_debug_node',
                output='screen'
            ),
        ]),

        # ── 12. Fire node ─────────────────────────────────────────────────
        TimerAction(period=8.0, actions=[
            Node(
                package='ship_control',
                executable='fire_node',
                parameters=[bridge_file],
                output='screen'
            ),
        ]),

    ])