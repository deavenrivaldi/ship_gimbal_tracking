"""
IMU COMPARISON DASHBOARD NODE
Package    : ship_vision

Purpose:
    Publishes a real-time dual-panel debug image to Foxglove that shows:
    - Panel A (left):  Time-series plot of boat IMU roll/pitch/yaw
    - Panel B (right): Time-series plot of camera IMU roll/pitch/yaw
    - Panel C (bottom): Live angle gauges comparing boat vs camera

    The goal is visual proof that the gimbal stabilizer works:
    the boat line swings with waves, the camera line stays flat near 0.

Subscribes:
    /imu/data                   (sensor_msgs/Imu)  — boat
    /camera_imu/data            (sensor_msgs/Imu)  — camera
    /debug/stabilizer_status    (geometry_msgs/Vector3)  — from stabilizer node

Publishes:
    /debug/imu_comparison       (sensor_msgs/Image) → Foxglove
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from collections import deque
import time


# ─────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────

HISTORY_LEN  = 200     # number of samples in the rolling window
CANVAS_W     = 1280
CANVAS_H     = 720
PUBLISH_HZ   = 15.0

# Plot Y-axis range in degrees
PLOT_Y_RANGE = 25.0    # ±25°

# Colors (BGR)
COLOR_ROLL   = (80,  200, 80)    # green
COLOR_PITCH  = (80,  150, 255)   # blue
COLOR_YAW    = (200, 80,  200)   # magenta
COLOR_ZERO   = (100, 100, 100)   # grey reference line
COLOR_BOAT   = (60,  160, 240)   # boat panel accent
COLOR_CAM    = (60,  220, 130)   # camera panel accent
COLOR_BG     = (18,  18,  22)    # near-black background
COLOR_GRID   = (35,  35,  42)
COLOR_TEXT   = (220, 220, 220)
COLOR_DIM    = (90,  90,  100)


def quat_to_rpy_deg(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


class ImuDashboardNode(Node):

    def __init__(self):
        super().__init__('imu_dashboard_node')
        self.bridge = CvBridge()

        # ── History buffers ──────────────────────────────────────────────
        self.boat_roll  = deque(maxlen=HISTORY_LEN)
        self.boat_pitch = deque(maxlen=HISTORY_LEN)
        self.boat_yaw   = deque(maxlen=HISTORY_LEN)

        self.cam_roll   = deque(maxlen=HISTORY_LEN)
        self.cam_pitch  = deque(maxlen=HISTORY_LEN)
        self.cam_yaw    = deque(maxlen=HISTORY_LEN)

        # Latest scalar values for the gauges
        self.latest_boat_roll  = 0.0
        self.latest_boat_pitch = 0.0
        self.latest_cam_roll   = 0.0
        self.latest_cam_pitch  = 0.0

        self.boat_imu_ok = False
        self.cam_imu_ok  = False

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Imu,     '/imu/data',        self._boat_cb,   10)
        self.create_subscription(Imu,     '/camera_imu/data', self._cam_cb,    10)
        self.create_subscription(Vector3, '/debug/stabilizer_status', self._status_cb, 10)

        # ── Publisher ────────────────────────────────────────────────────
        self.pub = self.create_publisher(Image, '/debug/imu_comparison', 10)

        # ── Draw timer ───────────────────────────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._draw_and_publish)

        self.get_logger().info('✅ ImuDashboardNode ready → /debug/imu_comparison')

    # ─────────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────────

    def _boat_cb(self, msg: Imu):
        r, p, y = quat_to_rpy_deg(msg.orientation)
        self.boat_roll.append(r)
        self.boat_pitch.append(p)
        self.boat_yaw.append(y)
        self.latest_boat_roll  = r
        self.latest_boat_pitch = p
        self.boat_imu_ok = True

    def _cam_cb(self, msg: Imu):
        r, p, y = quat_to_rpy_deg(msg.orientation)
        self.cam_roll.append(r)
        self.cam_pitch.append(p)
        self.cam_yaw.append(y)
        self.latest_cam_roll  = r
        self.latest_cam_pitch = p
        self.cam_imu_ok = True

    def _status_cb(self, msg: Vector3):
        # Stabilizer node already filters — use for gauges if IMU direct isn't available
        if not self.boat_imu_ok:
            self.latest_boat_roll  = msg.x
            self.latest_boat_pitch = msg.y
        if not self.cam_imu_ok:
            self.latest_cam_roll = msg.z

    # ─────────────────────────────────────────
    # DRAW
    # ─────────────────────────────────────────

    def _draw_and_publish(self):
        canvas = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
        canvas[:] = COLOR_BG

        # Layout:
        #  ┌─────────────────┬─────────────────┐
        #  │  BOAT IMU plot  │  CAMERA IMU plot │
        #  │                 │                  │
        #  ├─────────────────┴──────────────────┤
        #  │           Gauge row                │
        #  └────────────────────────────────────┘
        plot_h   = int(CANVAS_H * 0.60)
        gauge_h  = CANVAS_H - plot_h
        half_w   = CANVAS_W // 2

        self._draw_timeseries(canvas, 0,      0,       half_w, plot_h,
                              self.boat_roll, self.boat_pitch,
                              label='BOAT IMU', accent=COLOR_BOAT,
                              ok=self.boat_imu_ok)

        self._draw_timeseries(canvas, half_w, 0,       half_w, plot_h,
                              self.cam_roll, self.cam_pitch,
                              label='CAMERA IMU  (should be flat)', accent=COLOR_CAM,
                              ok=self.cam_imu_ok)

        # Vertical divider
        cv2.line(canvas, (half_w, 0), (half_w, plot_h), (50, 50, 60), 1)

        # Gauge row
        self._draw_gauges(canvas, 0, plot_h, CANVAS_W, gauge_h)

        # Watermark / title
        cv2.putText(canvas,
                    'GIMBAL STABILIZATION — IMU COMPARISON DASHBOARD',
                    (CANVAS_W // 2 - 340, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 1, cv2.LINE_AA)

        try:
            msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'publish error: {e}')

    # ─────────────────────────────────────────
    # PANEL: time-series plot
    # ─────────────────────────────────────────

    def _draw_timeseries(self, canvas, ox, oy, w, h,
                         roll_buf, pitch_buf,
                         label, accent, ok):
        """Draw a roll/pitch time-series strip at offset (ox, oy)."""
        PAD = 40
        ph = h - PAD * 2   # plot inner height
        pw = w - PAD - 10  # plot inner width

        # Background fill
        canvas[oy:oy+h, ox:ox+w] = COLOR_BG

        # Grid lines at ±5, ±10, ±15, ±20 deg
        for deg in range(-20, 25, 5):
            py = oy + PAD + int(ph * (1.0 - (deg + PLOT_Y_RANGE) / (2 * PLOT_Y_RANGE)))
            col = (50, 50, 70) if deg != 0 else (80, 80, 90)
            cv2.line(canvas, (ox + PAD, py), (ox + PAD + pw, py), col, 1)
            if deg % 10 == 0:
                cv2.putText(canvas, f'{deg:+d}°',
                            (ox + 2, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, COLOR_DIM, 1)

        # Zero line slightly brighter
        py0 = oy + PAD + int(ph * 0.5)
        cv2.line(canvas, (ox + PAD, py0), (ox + PAD + pw, py0), COLOR_ZERO, 1)

        # Draw roll and pitch traces
        for buf, color, name in [(roll_buf, COLOR_ROLL, 'Roll'),
                                  (pitch_buf, COLOR_PITCH, 'Pitch')]:
            pts = list(buf)
            if len(pts) < 2:
                continue
            n = len(pts)
            coords = []
            for i, val in enumerate(pts):
                x = ox + PAD + int(pw * i / (HISTORY_LEN - 1))
                y = oy + PAD + int(ph * (1.0 - (val + PLOT_Y_RANGE) / (2 * PLOT_Y_RANGE)))
                y = max(oy + PAD, min(oy + PAD + ph, y))
                coords.append((x, y))
            for i in range(1, len(coords)):
                cv2.line(canvas, coords[i-1], coords[i], color, 2, cv2.LINE_AA)

            # Live value label at right edge
            if coords:
                lx, ly = coords[-1]
                cv2.putText(canvas, f'{name}: {pts[-1]:+.1f}°',
                            (ox + PAD + pw - 130, ly + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

        # Panel label
        cv2.putText(canvas, label,
                    (ox + PAD, oy + PAD - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, accent, 2, cv2.LINE_AA)

        # "NO DATA" overlay
        if not ok:
            cv2.putText(canvas, 'WAITING FOR IMU…',
                        (ox + PAD + pw // 2 - 80, oy + h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 80), 2)

        # Legend
        lx = ox + PAD
        ly = oy + h - 12
        cv2.circle(canvas, (lx + 6, ly - 3), 4, COLOR_ROLL,  -1)
        cv2.putText(canvas, 'Roll', (lx + 14, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.36, COLOR_ROLL,  1)
        cv2.circle(canvas, (lx + 60, ly - 3), 4, COLOR_PITCH, -1)
        cv2.putText(canvas, 'Pitch', (lx + 68, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.36, COLOR_PITCH, 1)

    # ─────────────────────────────────────────
    # PANEL: gauges
    # ─────────────────────────────────────────

    def _draw_gauges(self, canvas, ox, oy, w, h):
        """Bottom row: side-by-side semicircular gauges for roll + pitch."""
        cx_boat_roll  = ox + w // 8
        cx_boat_pitch = ox + w * 3 // 8
        cx_cam_roll   = ox + w * 5 // 8
        cx_cam_pitch  = ox + w * 7 // 8
        cy = oy + h // 2 + 10
        r  = min(h // 2 - 15, 70)

        pairs = [
            (cx_boat_roll,  cy, self.latest_boat_roll,  'Boat Roll',   COLOR_BOAT),
            (cx_boat_pitch, cy, self.latest_boat_pitch, 'Boat Pitch',  COLOR_BOAT),
            (cx_cam_roll,   cy, self.latest_cam_roll,   'Cam Roll',    COLOR_CAM),
            (cx_cam_pitch,  cy, self.latest_cam_pitch,  'Cam Pitch',   COLOR_CAM),
        ]

        # Divider line above gauge row
        cv2.line(canvas, (ox, oy), (ox + w, oy), (45, 45, 55), 1)

        for cx, cy_, val, lbl, accent in pairs:
            self._draw_single_gauge(canvas, cx, cy_, r, val, lbl, accent)

        # Stabilization quality indicator
        cam_err = math.sqrt(self.latest_cam_roll**2 + self.latest_cam_pitch**2)
        if cam_err < 1.0:
            quality_color = (60, 220, 60)
            quality_text  = f'✓ STABLE  cam err={cam_err:.2f}°'
        elif cam_err < 4.0:
            quality_color = (60, 200, 220)
            quality_text  = f'~ SETTLING  cam err={cam_err:.2f}°'
        else:
            quality_color = (60, 60, 220)
            quality_text  = f'✗ UNSTABLE  cam err={cam_err:.2f}°'

        cv2.putText(canvas, quality_text,
                    (ox + w // 2 - 140, oy + h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, quality_color, 2, cv2.LINE_AA)

    def _draw_single_gauge(self, canvas, cx, cy, r, value_deg, label, accent):
        """Draw one semicircular gauge needle."""
        # Clamp display to ±PLOT_Y_RANGE
        disp = max(-PLOT_Y_RANGE, min(PLOT_Y_RANGE, value_deg))

        # Arc: 0° at left (180°), 180° at right (0°) in OpenCV coords
        # We map -PLOT_Y_RANGE → leftmost, 0 → top, +PLOT_Y_RANGE → rightmost
        cv2.ellipse(canvas, (cx, cy), (r, r), 0, 180, 360,
                    (45, 45, 55), 8)

        # Colour zones: ±5° green, ±10° yellow, beyond red
        for start_d, end_d, col in [
            (-20, -10, (40, 40, 180)),
            (-10,  -5, (40, 120, 220)),
            ( -5,   5, (40, 180, 80)),
            (  5,  10, (40, 120, 220)),
            ( 10,  20, (40, 40, 180)),
        ]:
            cv_s = int(180 + start_d / PLOT_Y_RANGE * 90)
            cv_e = int(180 + end_d   / PLOT_Y_RANGE * 90)
            cv2.ellipse(canvas, (cx, cy), (r, r), 0, cv_s, cv_e, col, 6)

        # Tick marks
        for d in range(-20, 25, 5):
            angle_cv = 180 + d / PLOT_Y_RANGE * 90
            rad = math.radians(angle_cv)
            x_o = int(cx + r * math.cos(rad))
            y_o = int(cy + r * math.sin(rad))
            x_i = int(cx + (r - 8) * math.cos(rad))
            y_i = int(cy + (r - 8) * math.sin(rad))
            cv2.line(canvas, (x_o, y_o), (x_i, y_i), (120, 120, 130), 1)

        # Needle
        angle_cv = 180 + disp / PLOT_Y_RANGE * 90
        rad = math.radians(angle_cv)
        nx = int(cx + (r - 4) * math.cos(rad))
        ny = int(cy + (r - 4) * math.sin(rad))
        tip_col = (60, 200, 60) if abs(disp) < 5 else (60, 60, 200)
        cv2.line(canvas, (cx, cy), (nx, ny), (220, 220, 220), 2, cv2.LINE_AA)
        cv2.circle(canvas, (cx, cy), 5, accent, -1)
        cv2.circle(canvas, (nx, ny), 4, tip_col, -1)

        # Value text
        cv2.putText(canvas, f'{value_deg:+.1f}°',
                    (cx - 25, cy + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, accent, 1, cv2.LINE_AA)
        # Label
        cv2.putText(canvas, label,
                    (cx - 30, cy - r - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLOR_DIM, 1)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()