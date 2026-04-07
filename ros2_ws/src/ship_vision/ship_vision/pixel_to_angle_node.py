"""
PIXEL TO ANGLE CONVERTER NODE
Package : ship_vision
Subscribes : /target/pixel_center  (geometry_msgs/Point)
Publishes  : /gimbal/angle_command  (geometry_msgs/Vector3)
             x = pan  angle degrees (+ RIGHT, - LEFT)
             y = tilt angle degrees (+ DOWN,  - UP)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3

# ------- CAMERA CONFIGURATION -------
# Must match SDF <image> and <horizontal_fov> exactly
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
FOV_H_DEG     = 60.0
FOV_V_DEG     = FOV_H_DEG * (CAMERA_HEIGHT / CAMERA_WIDTH)  # 45.0°

# ------- DEADBAND -------
DEADBAND_PX = 5

# ------- TIMEOUT -------
NO_TARGET_TIMEOUT = 0.5   # seconds before declaring target lost


class PixelToAngleNode(Node):

    def __init__(self):
        super().__init__('pixel_to_angle_node')

        self.deg_per_px_h = FOV_H_DEG / CAMERA_WIDTH
        self.deg_per_px_v = FOV_V_DEG / CAMERA_HEIGHT
        self.center_x     = CAMERA_WIDTH  // 2
        self.center_y     = CAMERA_HEIGHT // 2

        self.last_target_time = None
        self.target_active    = False

        self.get_logger().info(
            f'📐 Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT} | '
            f'FOV: {FOV_H_DEG}°x{FOV_V_DEG:.1f}° | '
            f'Resolution: {self.deg_per_px_h:.4f} deg/px'
        )
        self.get_logger().info(f'🎯 Deadband: ±{DEADBAND_PX}px')

        # Subscriber
        self.sub = self.create_subscription(
            Point,
            '/target/pixel_center',
            self.pixel_callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(
            Vector3,
            '/gimbal/angle_command',
            10
        )

        # Timeout checker
        self.timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('✅ PixelToAngleNode ready — waiting for targets...')


    def pixel_callback(self, msg):
        self.last_target_time = self.get_clock().now()

        if not self.target_active:
            self.get_logger().info('🟢 Target acquired — entering frame')
            self.target_active = True

        # Pixel offset from crosshair center
        offset_x = msg.x - self.center_x
        offset_y = msg.y - self.center_y

        # Apply deadband
        if abs(offset_x) < DEADBAND_PX:
            offset_x = 0.0
        if abs(offset_y) < DEADBAND_PX:
            offset_y = 0.0

        # Convert to angles
        pan_deg  = offset_x * self.deg_per_px_h
        tilt_deg = offset_y * self.deg_per_px_v

        # Publish
        cmd     = Vector3()
        cmd.x   = pan_deg
        cmd.y   = tilt_deg
        cmd.z   = 0.0
        self.pub.publish(cmd)

        # Log with directions
        if offset_x != 0.0 or offset_y != 0.0:
            pan_dir  = "RIGHT" if pan_deg  > 0 else "LEFT"
            tilt_dir = "DOWN"  if tilt_deg > 0 else "UP"
            self.get_logger().info(
                f'📐 Pixel: ({msg.x:.0f}, {msg.y:.0f}) | '
                f'Offset: ({offset_x:.0f}px, {offset_y:.0f}px) | '
                f'pan={abs(pan_deg):.2f}° {pan_dir}  '
                f'tilt={abs(tilt_deg):.2f}° {tilt_dir}',
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().info(
                '✅ Target centered — within deadband',
                throttle_duration_sec=1.0
            )


    def check_timeout(self):
        if self.last_target_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if elapsed > NO_TARGET_TIMEOUT and self.target_active:
            self.target_active = False
            self.get_logger().info('🔴 Target lost — outside frame, no output')


def main(args=None):
    rclpy.init(args=args)
    node = PixelToAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()