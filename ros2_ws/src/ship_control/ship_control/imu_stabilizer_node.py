"""
IMU STABILIZER NODE
Package    : ship_control

Purpose:
    Reads raw IMU data (quaternion orientation + angular velocity)
    Converts to roll/pitch angles in degrees
    Publishes correction angles that the gimbal needs to apply
    to keep the camera level regardless of boat motion

On flat ground:
    roll  = 0°  → correction = 0° → no gimbal roll movement
    pitch = 0°  → correction = 0° → no gimbal tilt compensation

On a boat with waves:
    roll  = +15° → correction = -15° → gimbal rolls -15° to stay level
    pitch = +5°  → correction = -5°  → gimbal tilt compensates +5°

Subscribes : /imu/data                  (sensor_msgs/Imu)
Publishes  : /gimbal/roll_correction    (geometry_msgs/Vector3)
                 x = roll_correction_deg
                 y = pitch_correction_deg
                 z = 0 (reserved)
             /debug/imu_angles          (geometry_msgs/Vector3)
                 x = raw_roll_deg
                 y = raw_pitch_deg
                 z = raw_yaw_deg
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math


# ------- CONFIGURATION -------

# Low-pass filter coefficient (0-1)
# Higher = smoother but slower response
# Lower  = faster but noisier
# 0.85 is good for boat wave frequencies
FILTER_ALPHA = 0.85

# Threshold below which we ignore correction (deg)
# Prevents micro-corrections from sensor noise
CORRECTION_DEADBAND = 0.5


class ImuStabilizerNode(Node):

    def __init__(self):
        super().__init__('imu_stabilizer_node')

        # Filtered angle values
        self.filtered_roll  = 0.0
        self.filtered_pitch = 0.0
        self.filtered_yaw   = 0.0

        # First reading flag for filter initialization
        self.initialized = False

        # Subscriber
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher 1 — correction angles for gimbal controller
        self.pub_correction = self.create_publisher(
            Vector3,
            '/gimbal/roll_correction',
            10
        )

        # Publisher 2 — raw angles for debug/plotting
        self.pub_debug = self.create_publisher(
            Vector3,
            '/debug/imu_angles',
            10
        )

        self.get_logger().info(
            f'✅ ImuStabilizerNode ready!\n'
            f'   Filter alpha : {FILTER_ALPHA}\n'
            f'   Deadband     : ±{CORRECTION_DEADBAND}°\n'
            f'   On flat ground: roll=0°, pitch=0° → no correction'
        )


    def imu_callback(self, msg):
        """Convert quaternion → roll/pitch/yaw, apply filter, publish."""

        # Step 1 — extract quaternion
        q = msg.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z

        # Step 2 — quaternion → Euler angles (radians)
        # Roll (rotation around X axis)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        raw_roll  = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (rotation around Y axis)
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))   # clamp to [-1, 1]
        raw_pitch = math.asin(sinp)

        # Yaw (rotation around Z axis)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        raw_yaw   = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        raw_roll_deg  = math.degrees(raw_roll)
        raw_pitch_deg = math.degrees(raw_pitch)
        raw_yaw_deg   = math.degrees(raw_yaw)

        # Step 3 — low-pass filter
        # First reading: initialize filter without blending
        if not self.initialized:
            self.filtered_roll  = raw_roll_deg
            self.filtered_pitch = raw_pitch_deg
            self.filtered_yaw   = raw_yaw_deg
            self.initialized    = True
        else:
            self.filtered_roll  = (FILTER_ALPHA * self.filtered_roll +
                                   (1 - FILTER_ALPHA) * raw_roll_deg)
            self.filtered_pitch = (FILTER_ALPHA * self.filtered_pitch +
                                   (1 - FILTER_ALPHA) * raw_pitch_deg)
            self.filtered_yaw   = (FILTER_ALPHA * self.filtered_yaw +
                                   (1 - FILTER_ALPHA) * raw_yaw_deg)

        # Step 4 — compute correction
        # Gimbal must rotate OPPOSITE to boat tilt to stay level
        roll_correction  = -self.filtered_roll
        pitch_correction = -self.filtered_pitch

        # Apply deadband — ignore tiny corrections from sensor noise
        if abs(roll_correction)  < CORRECTION_DEADBAND:
            roll_correction  = 0.0
        if abs(pitch_correction) < CORRECTION_DEADBAND:
            pitch_correction = 0.0

        # Step 5 — publish correction
        correction_msg   = Vector3()
        correction_msg.x = roll_correction
        correction_msg.y = pitch_correction
        correction_msg.z = 0.0
        self.pub_correction.publish(correction_msg)

        # Step 6 — publish raw angles for debug
        debug_msg   = Vector3()
        debug_msg.x = self.filtered_roll
        debug_msg.y = self.filtered_pitch
        debug_msg.z = self.filtered_yaw
        self.pub_debug.publish(debug_msg)

        # Log every 1 second
        self.get_logger().info(
            f'📡 IMU — '
            f'roll:{self.filtered_roll:+.2f}°  '
            f'pitch:{self.filtered_pitch:+.2f}°  '
            f'yaw:{self.filtered_yaw:+.2f}° | '
            f'Correction — '
            f'roll:{roll_correction:+.2f}°  '
            f'pitch:{pitch_correction:+.2f}°',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()