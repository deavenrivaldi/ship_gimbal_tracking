"""
GIMBAL CONTROLLER NODE
Package    : ship_control
Subscribes : /gimbal/angle_command    (geometry_msgs/Vector3) ← vision
             /gimbal/roll_correction  (geometry_msgs/Vector3) ← IMU
Publishes  : /gimbal/joint_trajectory    (trajectory_msgs/JointTrajectory)

Combined control logic:
    pan_error  = pan_cmd              (vision only)
    tilt_error = tilt_cmd + pitch_correction  (vision + IMU)
    roll_error = roll_correction      (IMU only — stabilization)
"""

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3,  Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityWrench, Entity
from ship_msgs.srv import Fire
from ament_index_python.packages import get_package_share_directory


# ------- METADATA -------
DEADBAND_DEG  = 0.5

# Timeout — if no vision command received, hold position
VISION_TIMEOUT = 0.5


class GimbalControllerNode(Node):

    JOINT_NAMES = ['yaw_joint', 'pitch_joint', 'roll_joint']
    JOINT_LIMITS = {
        'yaw_joint':   (-3.14, 3.14),
        'pitch_joint':(-1.57, 1.57),
        'roll_joint': (-1.57, 1.57),
    }
    TIME_FROM_START = 0.10

    def __init__(self):
        super().__init__('gimbal_controller_node')

        # --- 1. 參數配置 (可在啟動時透過 yaml 或命令列覆蓋) ---
        self.declare_parameter('gimbal_yaw', 0.0)
        self.declare_parameter('gimbal_pitch', 0.0)
        self.declare_parameter('gimbal_roll', 0.0)
        self.declare_parameter('world_name', 'gimbal_world')
        self.declare_parameter('projectile_package', 'ship_simulation')
        self.declare_parameter('projectile_sdf', 'models/bullet/projectile_sphere.sdf')

        self.world_name = self.get_parameter('world_name').value
        self.projectile_package = self.get_parameter('projectile_package').value
        self.projectile_sdf = self.get_parameter('projectile_sdf').value
        self.projectile_path = os.path.join(
            get_package_share_directory(self.projectile_package),
            self.projectile_sdf
        )

        # Vision inputs (from pixel_to_angle)
        self.pan_cmd  = 0.0
        self.tilt_cmd = 0.0
        self.vision_active    = False
        self.last_vision_time = None

        # IMU inputs (from imu_stabilizer)
        self.roll_correction  = 0.0
        self.pitch_correction = 0.0
        self.imu_active = False

        # Joint positions and command setpoints
        self.current_positions = {name: 0.0 for name in self.JOINT_NAMES}
        self.target_positions  = dict(self.current_positions)
        self.joint_state_available = False

        self.last_time = self.get_clock().now()

        # continuous ratation
        self.add = {
            'yaw_joint': True,
            'pitch_joint': True,
            'roll_joint': True,
        }
        
        # Gimbal rotation direction flags
        self.yaw_add = True
        self.pitch_add = True
        self.roll_add = True

        # Fire service and projectile spawn/wrench interface
        self.fire_service = self.create_service(
            Fire,
            '/fire',
            self.fire_callback
        )

        self.spawn_client = self.create_client(
            SpawnEntity,
            f'/world/{self.world_name}/create'
        )
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(f"SpawnEntity service not available at /world/{self.world_name}/create")

        self.wrench_publisher = self.create_publisher(
            EntityWrench,
            f'/world/{self.world_name}/wrench',
            10
        )

        self.pending_projectiles = {}

        # ------- Subscribers -------
        # self.sub_vision = self.create_subscription(
        #     Vector3,
        #     '/gimbal/angle_command',
        #     self.vision_callback,
        #     10
        # )

        # self.sub_imu = self.create_subscription(
        #     Vector3,
        #     '/gimbal/roll_correction',
        #     self.imu_correction_callback,
        #     10
        # )

        # self.sub_joints = self.create_subscription(
        #     JointState,
        #     '/world/gimbal_world/model/gimbal/joint_state',
        #     self.joint_state_callback,
        #     10
        # )

        # ------- Publishers -------
        self.pub_trajectory = self.create_publisher(
            JointTrajectory,
            '/gimbal/joint_trajectory',
            10
        )

        # Control loop at 20Hz
        # self.timer = self.create_timer(0.05, self.control_loop)
        self.timer = self.create_timer(0.2, self.test_loop)

        self.get_logger().info(
            f'✅ GimbalControllerNode ready!\n'
            f'   Publisher: /gimbal/joint_trajectory\n'
            f'   Service: /fire\n'
            f'   Deadband: ±{DEADBAND_DEG}°'
        )


    def fire_callback(self, request, response):
        self.get_logger().info('Fire request received')

        name = self.shoot_projectile(
            request.position,
            request.direction,
            request.force
        )

        if name is None:
            response.success = False
            response.projectile_name = ''
            return response

        response.success = True
        response.projectile_name = name
        return response


    def shoot_projectile(self, position: Point, direction: Vector3, force: float):
        """
        position: geometry_msgs.msg.Point
        direction: geometry_msgs.msg.Vector3
        force: float
        """
        if not os.path.exists(self.projectile_path):
            self.get_logger().error(f'Projectile SDF not found: {self.projectile_path}')
            return None

        name = f'projectile_{self.get_clock().now().nanoseconds}'
        spawn_request = SpawnEntity.Request()
        spawn_request.entity_factory.sdf_filename = self.projectile_path
        spawn_request.entity_factory.name = name
        spawn_request.entity_factory.pose.position = position

        future = self.spawn_client.call_async(spawn_request)
        self.pending_projectiles[future] = {
            'name': name,
            'direction': direction,
            'force': force,
        }
        future.add_done_callback(self.spawn_response_callback)
        return name


    def spawn_response_callback(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Projectile spawn failed: {exc}')
            return

        context = self.pending_projectiles.pop(future, None)
        if context is None or not result.success:
            self.get_logger().error('Spawn failed or missing spawn context')
            return

        name = context['name']
        direction = context['direction']
        force = context['force']

        self.get_logger().info(f'Spawn OK: {name}')

        wrench_msg = EntityWrench()
        wrench_msg.entity.name = name
        wrench_msg.entity.type = Entity.MODEL
        wrench_msg.wrench.force.x = direction.x * force
        wrench_msg.wrench.force.y = direction.y * force
        wrench_msg.wrench.force.z = direction.z * force

        self.wrench_publisher.publish(wrench_msg)
        self.get_logger().info(f'Projectile {name} fired')


    def vision_callback(self, msg):
        """Vision tracking input — pan and tilt errors from pixel_to_angle."""
        self.pan_cmd  = math.radians(msg.x)
        self.tilt_cmd = math.radians(msg.y)
        self.vision_active    = True
        self.last_vision_time = self.get_clock().now()


    def imu_correction_callback(self, msg):
        """IMU stabilization input — roll and pitch corrections."""
        self.roll_correction  = math.radians(msg.x)
        self.pitch_correction = math.radians(msg.y)
        if not self.imu_active:
            self.imu_active = True
            self.get_logger().info('📡 IMU correction active')


    def joint_state_callback(self, msg):
        """Read current joint positions from Gazebo joint_state."""
        for name, position in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = position
                self.joint_state_available = True


    def check_vision_timeout(self):
        """Detect when target leaves frame."""
        if self.last_vision_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_vision_time).nanoseconds / 1e9
        if elapsed > VISION_TIMEOUT and self.vision_active:
            self.vision_active = False
            self.pan_cmd  = 0.0
            self.tilt_cmd = 0.0
            self.get_logger().info('🔴 Vision lost — holding position, IMU stabilizing')


    def build_trajectory(self, positions, duration_sec):
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [positions[name] for name in self.JOINT_NAMES]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9)
        )

        traj.points = [point]
        return traj


    def control_loop(self):
        """
        Build a joint trajectory command from vision and IMU inputs.
        """
        self.check_vision_timeout()

        pan_error  = self.pan_cmd
        tilt_error = self.tilt_cmd + self.pitch_correction
        roll_error = self.roll_correction

        pan_error  = pan_error  if abs(math.degrees(pan_error))  > DEADBAND_DEG else 0.0
        tilt_error = tilt_error if abs(math.degrees(tilt_error)) > DEADBAND_DEG else 0.0
        roll_error = roll_error if abs(math.degrees(roll_error)) > DEADBAND_DEG else 0.0

        # Update target positions based on errors
        yaw_target = self.get_parameter('gimbal_yaw').value + pan_error
        pitch_target = self.get_parameter('gimbal_pitch').value + tilt_error
        roll_target = self.get_parameter('gimbal_roll').value + roll_error

        # Apply joint limits
        yaw_target = max(self.JOINT_LIMITS['yaw_joint'][0], min(self.JOINT_LIMITS['yaw_joint'][1], yaw_target))
        pitch_target = max(self.JOINT_LIMITS['pitch_joint'][0], min(self.JOINT_LIMITS['pitch_joint'][1], pitch_target))
        roll_target = max(self.JOINT_LIMITS['roll_joint'][0], min(self.JOINT_LIMITS['roll_joint'][1], roll_target))

        # Update parameters
        new_params = [
            Parameter('gimbal_yaw', Parameter.Type.DOUBLE, yaw_target),
            Parameter('gimbal_pitch', Parameter.Type.DOUBLE, pitch_target),
            Parameter('gimbal_roll', Parameter.Type.DOUBLE, roll_target)
        ]
        self.set_parameters(new_params)

        # Build and publish trajectory
        positions = {
            'yaw_joint': yaw_target,
            'pitch_joint': pitch_target,
            'roll_joint': roll_target
        }
        trajectory = self.build_trajectory(positions, self.TIME_FROM_START)
        self.pub_trajectory.publish(trajectory)

        self.get_logger().info(
            f'🎮 pan:{math.degrees(pan_error):+.1f}° '
            f'tilt:{math.degrees(tilt_error):+.1f}° '
            f'roll:{math.degrees(roll_error):+.1f}°  '
            f'[vision:{"✅" if self.vision_active else "❌"} '
            f'imu:{"✅" if self.imu_active else "❌"}]',
            throttle_duration_sec=0.5
        )


    def test_loop(self):
        """Test gimbal continuous rotation within joint limits."""
        deg = 0.05  # 0.05 radians per cycle

        # Calculate target positions with continuous rotation
        yaw_target = self.get_parameter('gimbal_yaw').value + (deg if self.yaw_add else -deg)
        pitch_target = self.get_parameter('gimbal_pitch').value + (deg if self.pitch_add else -deg)
        roll_target = self.get_parameter('gimbal_roll').value + (deg if self.roll_add else -deg)

        # Check and apply joint limits with direction reversal
        if yaw_target > self.JOINT_LIMITS['yaw_joint'][1] or yaw_target < self.JOINT_LIMITS['yaw_joint'][0]:
            self.yaw_add = not self.yaw_add
            self.shoot_projectile(Point(x=0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=1.0), 150.0)
        if pitch_target > self.JOINT_LIMITS['pitch_joint'][1] or pitch_target < self.JOINT_LIMITS['pitch_joint'][0]:
            self.pitch_add = not self.pitch_add
        if roll_target > self.JOINT_LIMITS['roll_joint'][1] or roll_target < self.JOINT_LIMITS['roll_joint'][0]:
            self.roll_add = not self.roll_add

        # Clamp target positions to joint limits
        yaw_target = max(self.JOINT_LIMITS['yaw_joint'][0], min(self.JOINT_LIMITS['yaw_joint'][1], yaw_target))
        pitch_target = max(self.JOINT_LIMITS['pitch_joint'][0], min(self.JOINT_LIMITS['pitch_joint'][1], pitch_target))
        roll_target = max(self.JOINT_LIMITS['roll_joint'][0], min(self.JOINT_LIMITS['roll_joint'][1], roll_target))

        # Update parameters
        new_params = [
            Parameter('gimbal_yaw', Parameter.Type.DOUBLE, yaw_target),
            Parameter('gimbal_pitch', Parameter.Type.DOUBLE, pitch_target),
            Parameter('gimbal_roll', Parameter.Type.DOUBLE, roll_target)
        ]
        self.set_parameters(new_params)

        # Build trajectory command
        positions = {
            'yaw_joint': yaw_target,
            'pitch_joint': pitch_target,
            'roll_joint': roll_target
        }
        trajectory = self.build_trajectory(positions, self.TIME_FROM_START)
        self.pub_trajectory.publish(trajectory)

        # Update and log current target positions
        self.target_positions = positions
        self.get_logger().info(
            f"yaw_joint:{math.degrees(self.target_positions['yaw_joint']):+.1f}° "
            f"pitch_joint:{math.degrees(self.target_positions['pitch_joint']):+.1f}° "
            f"roll_joint:{math.degrees(self.target_positions['roll_joint']):+.1f}°",
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()