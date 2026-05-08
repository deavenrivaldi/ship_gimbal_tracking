"""
GIMBAL CONTROLLER NODE
Package    : ship_control
Subscribes : /gimbal/angle_command    (geometry_msgs/Vector3) ← vision
             /gimbal/roll_correction  (geometry_msgs/Vector3) ← IMU
Publishes  : /gimbal/pan/cmd_vel      (std_msgs/Float64)
             /gimbal/tilt/cmd_vel     (std_msgs/Float64)
             /gimbal/roll/cmd_vel     (std_msgs/Float64)

Combined control logic:
    pan_error  = pan_cmd              (vision only)
    tilt_error = tilt_cmd + pitch_correction  (vision + IMU)
    roll_error = roll_correction      (IMU only — stabilization)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math


# ------- PID CONFIGURATION -------
PAN_PID  = {'Kp': 0.05, 'Ki': 0.001, 'Kd': 0.008}
TILT_PID = {'Kp': 0.05, 'Ki': 0.001, 'Kd': 0.008}
ROLL_PID = {'Kp': 0.05, 'Ki': 0.001, 'Kd': 0.008}

MAX_VEL       = 1.0
DEADBAND_DEG  = 0.5
INTEGRAL_LIMIT = 10.0

# Timeout — if no vision command received, hold position
VISION_TIMEOUT = 0.5


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral   = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        P = self.Kp * error
        self.integral += error * dt
        self.integral  = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral))
        I = self.Ki * self.integral
        D = self.Kd * (error - self.prev_error) / dt
        self.prev_error = error
        output = P + I + D
        return max(-MAX_VEL, min(MAX_VEL, output))

    def reset(self):
        self.integral   = 0.0
        self.prev_error = 0.0


class GimbalControllerNode(Node):

    def __init__(self):
        super().__init__('gimbal_controller_node')

        self.pid_pan  = PIDController(**PAN_PID)
        self.pid_tilt = PIDController(**TILT_PID)
        self.pid_roll = PIDController(**ROLL_PID)

        # Vision inputs (from pixel_to_angle)
        self.pan_cmd  = 0.0
        self.tilt_cmd = 0.0
        self.vision_active    = False
        self.last_vision_time = None

        # IMU inputs (from imu_stabilizer)
        self.roll_correction  = 0.0
        self.pitch_correction = 0.0
        self.imu_active = False

        self.last_time = self.get_clock().now()

        # ------- Subscribers -------
        self.sub_vision = self.create_subscription(
            Vector3,
            '/gimbal/angle_command',
            self.vision_callback,
            10
        )

        self.sub_imu = self.create_subscription(
            Vector3,
            '/gimbal/roll_correction',
            self.imu_correction_callback,
            10
        )

        self.sub_joints = self.create_subscription(
            JointState,
            '/world/gimbal_world/model/gimbal/joint_state',
            self.joint_state_callback,
            10
        )

        # ------- Publishers -------
        self.pub_pan  = self.create_publisher(Float64, '/gimbal/pan/cmd_vel',  10)
        self.pub_tilt = self.create_publisher(Float64, '/gimbal/tilt/cmd_vel', 10)
        self.pub_roll = self.create_publisher(Float64, '/gimbal/roll/cmd_vel', 10)

        # Control loop at 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f'✅ GimbalControllerNode ready!\n'
            f'   PAN  PID: {PAN_PID}\n'
            f'   TILT PID: {TILT_PID}\n'
            f'   ROLL PID: {ROLL_PID}\n'
            f'   Deadband: ±{DEADBAND_DEG}°  MaxVel: {MAX_VEL} rad/s'
        )


    def vision_callback(self, msg):
        """Vision tracking input — pan and tilt errors from pixel_to_angle."""
        self.pan_cmd  = msg.x
        self.tilt_cmd = msg.y
        self.vision_active    = True
        self.last_vision_time = self.get_clock().now()


    def imu_correction_callback(self, msg):
        """IMU stabilization input — roll and pitch corrections."""
        self.roll_correction  = msg.x
        self.pitch_correction = msg.y
        if not self.imu_active:
            self.imu_active = True
            self.get_logger().info('📡 IMU correction active')


    def joint_state_callback(self, msg):
        """Read current joint positions — available for future use."""
        pass


    def check_vision_timeout(self):
        """Detect when target leaves frame."""
        if self.last_vision_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_vision_time).nanoseconds / 1e9
        if elapsed > VISION_TIMEOUT and self.vision_active:
            self.vision_active = False
            self.pan_cmd  = 0.0
            self.tilt_cmd = 0.0
            self.pid_pan.reset()
            self.pid_tilt.reset()
            self.get_logger().info('🔴 Vision lost — holding position, IMU stabilizing')


    def control_loop(self):
        """
        Combined PID control loop at 50Hz.

        Error combination:
            pan_error  = vision pan only
            tilt_error = vision tilt + IMU pitch compensation
            roll_error = IMU roll stabilization only
        """
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.check_vision_timeout()

        # Combine vision + IMU inputs
        pan_error  = self.pan_cmd
        tilt_error = self.tilt_cmd + self.pitch_correction
        roll_error = self.roll_correction

        # Apply deadband per axis
        pan_error  = pan_error  if abs(pan_error)  > DEADBAND_DEG else 0.0
        tilt_error = tilt_error if abs(tilt_error) > DEADBAND_DEG else 0.0
        roll_error = roll_error if abs(roll_error) > DEADBAND_DEG else 0.0

        # Compute PID
        pan_vel  = self.pid_pan.compute(pan_error,   dt)
        tilt_vel = self.pid_tilt.compute(tilt_error, dt)
        roll_vel = self.pid_roll.compute(roll_error, dt)

        # Publish
        self.pub_pan.publish(Float64(data=pan_vel))
        self.pub_tilt.publish(Float64(data=tilt_vel))
        self.pub_roll.publish(Float64(data=roll_vel))

        # Log
        self.get_logger().info(
            f'🎮 Pan:{pan_error:+.1f}°→{pan_vel:+.3f}  '
            f'Tilt:{tilt_error:+.1f}°→{tilt_vel:+.3f}  '
            f'Roll:{roll_error:+.1f}°→{roll_vel:+.3f} rad/s '
            f'[vision:{"✅" if self.vision_active else "❌"}  '
            f'imu:{"✅" if self.imu_active else "❌"}]',
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