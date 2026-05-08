"""
PLOT DEBUG NODE
Package    : ship_vision
Subscribes : /target/pixel_center   → X/Y targeting plot
             /gimbal/angle_command  → roll stability gauge
             /imu/data              → raw IMU roll (when available)
Publishes  : /debug/plot_image      → combined debug visualization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

# ------- CONFIGURATION -------
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
PLOT_SIZE     = 400   # XY plot canvas size (pixels)
GAUGE_SIZE    = 300   # Roll gauge canvas size (pixels)


class PlotDebugNode(Node):

    def __init__(self):
        super().__init__('plot_debug_node')
        self.bridge = CvBridge()

        # Latest data
        self.target_x    = 0.0
        self.target_y    = 0.0
        self.pan_deg     = 0.0
        self.tilt_deg    = 0.0
        self.roll_deg    = 0.0    # from angle_command z (or IMU)
        self.imu_roll    = 0.0   # raw IMU roll in degrees
        self.has_target  = False
        self.has_imu     = False

        # History trail for XY plot
        self.trail = []
        self.MAX_TRAIL = 60

        # Subscribers
        self.sub_pixel = self.create_subscription(
            Point, '/target/pixel_center',
            self.pixel_callback, 10
        )
        self.sub_angle = self.create_subscription(
            Vector3, '/gimbal/angle_command',
            self.angle_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback, 10
        )

        # Publisher
        self.pub_plot = self.create_publisher(
            Image, '/debug/plot_image', 10
        )

        # Timeout for target lost
        self.last_target_time = None
        self.TARGET_TIMEOUT   = 0.5

        # Draw timer — 15Hz
        self.timer = self.create_timer(1.0 / 15.0, self.draw_and_publish)

        self.get_logger().info('✅ PlotDebugNode ready!')


    def pixel_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.has_target = True
        self.last_target_time = self.get_clock().now()

        # Add to trail
        self.trail.append((msg.x, msg.y))
        if len(self.trail) > self.MAX_TRAIL:
            self.trail.pop(0)


    def angle_callback(self, msg):
        self.pan_deg  = msg.x
        self.tilt_deg = msg.y
        self.roll_deg = msg.z


    def imu_callback(self, msg):
        """Convert quaternion → roll in degrees."""
        self.has_imu = True
        q = msg.orientation
        # Roll from quaternion
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.imu_roll = math.degrees(math.atan2(sinr, cosr))


    def check_target_timeout(self):
        if self.last_target_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if elapsed > self.TARGET_TIMEOUT:
            self.has_target = False
            self.trail.clear()


    def draw_and_publish(self):
        self.check_target_timeout()

        # Draw both panels
        xy_plot    = self.draw_xy_plot()
        roll_gauge = self.draw_roll_gauge()

        # Stack side by side
        # Resize roll gauge to match xy_plot height
        gauge_resized = cv2.resize(
            roll_gauge,
            (int(roll_gauge.shape[1] * PLOT_SIZE / roll_gauge.shape[0]), PLOT_SIZE)
        )
        combined = np.hstack([xy_plot, gauge_resized])

        try:
            msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
            self.pub_plot.publish(msg)
        except Exception as e:
            self.get_logger().error(f'publish error: {e}')


    # ==========================================
    # PANEL 1 — XY Targeting Plot
    # ==========================================
    def draw_xy_plot(self):
        canvas = np.zeros((PLOT_SIZE, PLOT_SIZE, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)   # dark background

        cx = PLOT_SIZE // 2
        cy = PLOT_SIZE // 2
        scale = PLOT_SIZE / max(CAMERA_WIDTH, CAMERA_HEIGHT)

        # Grid lines
        for i in range(0, PLOT_SIZE, 50):
            cv2.line(canvas, (i, 0), (i, PLOT_SIZE), (40, 40, 40), 1)
            cv2.line(canvas, (0, i), (PLOT_SIZE, i), (40, 40, 40), 1)

        # Axis lines
        cv2.line(canvas, (cx, 0), (cx, PLOT_SIZE), (80, 80, 80), 1)
        cv2.line(canvas, (0, cy), (PLOT_SIZE, cy), (80, 80, 80), 1)

        # Camera frame boundary
        fw = int(CAMERA_WIDTH  * scale)
        fh = int(CAMERA_HEIGHT * scale)
        fx = cx - fw // 2
        fy = cy - fh // 2
        cv2.rectangle(canvas, (fx, fy), (fx + fw, fy + fh), (60, 60, 60), 1)

        # Deadband circle (5px deadband scaled)
        db_r = int(5 * scale)
        cv2.circle(canvas, (cx, cy), db_r, (0, 100, 100), 1)

        # Trail
        for i, (tx, ty) in enumerate(self.trail):
            px = int(cx + (tx - CAMERA_WIDTH  // 2) * scale)
            py = int(cy + (ty - CAMERA_HEIGHT // 2) * scale)
            alpha = int(255 * (i / len(self.trail)))
            cv2.circle(canvas, (px, py), 2, (0, alpha, 0), -1)

        # Crosshair (center)
        cv2.line(canvas, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
        cv2.line(canvas, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)
        cv2.circle(canvas, (cx, cy), 15, (255, 255, 255), 1)

        # Target dot
        if self.has_target:
            tx = int(cx + (self.target_x - CAMERA_WIDTH  // 2) * scale)
            ty = int(cy + (self.target_y - CAMERA_HEIGHT // 2) * scale)

            # Line from center to target
            cv2.line(canvas, (cx, cy), (tx, ty), (0, 200, 255), 1)

            # Target dot
            cv2.circle(canvas, (tx, ty), 7, (0, 255, 255), -1)
            cv2.circle(canvas, (tx, ty), 7, (255, 255, 255), 1)

            # Angle labels
            cv2.putText(canvas,
                f'pan:{self.pan_deg:+.1f}deg',
                (10, PLOT_SIZE - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(canvas,
                f'tilt:{self.tilt_deg:+.1f}deg',
                (10, PLOT_SIZE - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        else:
            cv2.putText(canvas, 'NO TARGET',
                (cx - 50, cy + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 180), 2)

        # Title
        cv2.putText(canvas, 'TARGET POSITION (X/Y)',
            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        return canvas


    # ==========================================
    # PANEL 2 — Roll Stability Gauge
    # ==========================================
    def draw_roll_gauge(self):
        canvas = np.zeros((GAUGE_SIZE, GAUGE_SIZE, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)

        cx = GAUGE_SIZE // 2
        cy = int(GAUGE_SIZE * 0.55)   # center slightly lower
        radius = int(GAUGE_SIZE * 0.38)

        # Use IMU roll if available, otherwise use roll_deg from angle_command
        display_roll = self.imu_roll if self.has_imu else 0.0
        # Convert to 0-180 display range (90 = perfectly level)
        display_val = 90.0 + display_roll   # -90°→0°, 0°→90°, +90°→180°
        display_val = max(0.0, min(180.0, display_val))

        # Draw gauge arc (180° semicircle)
        # Color zones:
        #   0-60°   : red (heavily tilted left)
        #   60-75°  : orange
        #   75-105° : green (stable zone around 90°)
        #   105-120°: orange
        #   120-180°: red (heavily tilted right)
        zones = [
            (0,   60,  (0, 0, 200),   'LEFT TILT'),
            (60,  75,  (0, 165, 255), ''),
            (75,  105, (0, 200, 0),   'STABLE'),
            (105, 120, (0, 165, 255), ''),
            (120, 180, (0, 0, 200),   'RIGHT TILT'),
        ]

        for start_deg, end_deg, color, _ in zones:
            # Convert to OpenCV arc angles (0° = right, CCW)
            # Gauge 0° is at left (180° in cv2), 180° is at right (0° in cv2)
            cv2_start = 180 - end_deg
            cv2_end   = 180 - start_deg
            cv2.ellipse(canvas, (cx, cy), (radius, radius),
                        0, cv2_start, cv2_end, color, 8)

        # Tick marks
        for angle_deg in range(0, 181, 15):
            rad = math.radians(180 - angle_deg)
            x_outer = int(cx + radius * math.cos(rad))
            y_outer = int(cy - radius * math.sin(rad))
            x_inner = int(cx + (radius - 12) * math.cos(rad))
            y_inner = int(cy - (radius - 12) * math.sin(rad))
            cv2.line(canvas, (x_outer, y_outer), (x_inner, y_inner),
                     (180, 180, 180), 2)

            # Label every 30°
            if angle_deg % 30 == 0:
                x_label = int(cx + (radius - 28) * math.cos(rad))
                y_label = int(cy - (radius - 28) * math.sin(rad))
                cv2.putText(canvas, str(angle_deg),
                    (x_label - 8, y_label + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)

        # 90° center mark (perfect level)
        rad_90 = math.radians(90)
        x90 = int(cx + (radius + 5) * math.cos(rad_90))
        y90 = int(cy - (radius + 5) * math.sin(rad_90))
        cv2.circle(canvas, (cx, int(cy - radius - 5)), 5, (0, 255, 0), -1)

        # Needle
        needle_rad = math.radians(180 - display_val)
        nx = int(cx + (radius - 5) * math.cos(needle_rad))
        ny = int(cy - (radius - 5) * math.sin(needle_rad))
        cv2.line(canvas, (cx, cy), (nx, ny), (255, 255, 255), 3)
        cv2.circle(canvas, (cx, cy), 8, (255, 255, 255), -1)

        # Needle tip color — green if stable, red if tilted
        tip_color = (0, 255, 0) if 75 <= display_val <= 105 else (0, 0, 255)
        cv2.circle(canvas, (nx, ny), 5, tip_color, -1)

        # Digital readout
        roll_text = f'{display_roll:+.1f}deg'
        stable_text = 'STABLE' if 75 <= display_val <= 105 else 'TILTED'
        stable_color = (0, 255, 0) if stable_text == 'STABLE' else (0, 0, 255)

        cv2.putText(canvas, roll_text,
            (cx - 30, cy + 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(canvas, stable_text,
            (cx - 35, cy + 55),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, stable_color, 2)

        # IMU source indicator
        source = 'IMU' if self.has_imu else 'ANGLE CMD'
        cv2.putText(canvas, f'src:{source}',
            (10, GAUGE_SIZE - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

        # Title
        cv2.putText(canvas, 'ROLL STABILITY',
            (cx - 55, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        return canvas


def main(args=None):
    rclpy.init(args=args)
    node = PlotDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()