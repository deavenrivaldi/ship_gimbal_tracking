"""
GIMBAL STABILIZER NODE  (v2 — camera-IMU closed loop)
Package    : ship_control

Control strategy:
    Close the loop ENTIRELY on the camera IMU.
    Goal: camera_roll = 0,  camera_pitch = 0  at all times.

    Error  = camera_roll_measured   (we want this to be 0)
    Output = joint position increment that reduces that error

    This is simpler and more robust than feedforward from the boat IMU:
    - It doesn't care what the boat is doing
    - It doesn't need the boat and camera IMU frames to be aligned
    - It self-corrects for any mechanical gimbal bias at rest
    - The boat IMU is still read and published for the dashboard, but
      it plays no part in the control calculation

    Why incremental (velocity-style PID) instead of absolute position?
    The gimbal joint controller accepts absolute position targets.
    We maintain a running `cmd_roll` / `cmd_pitch` and nudge it each
    step by the PID output.  This prevents the integral from causing
    position jumps and keeps the command within joint limits naturally.

Subscribes:
    /imu/data                   (sensor_msgs/Imu)  — boat IMU (display only)
    /camera_imu/data            (sensor_msgs/Imu)  — camera IMU (control)

Publishes:
    /gimbal/joint_trajectory    (trajectory_msgs/JointTrajectory)
    /gimbal/roll_correction     (geometry_msgs/Vector3)
    /debug/stabilizer_status    (geometry_msgs/Vector3)
        x = boat_roll_deg
        y = boat_pitch_deg
        z = camera_roll_deg   ← proof of stabilization, should be ~0
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ─────────────────────────────────────────────────────────────
# TUNING  — adjust these if the gimbal oscillates or is sluggish
# ─────────────────────────────────────────────────────────────

# How much to move the joint per degree of camera error (rad/deg).
# Think of this as "sensitivity".  Start low, increase until responsive.
ROLL_KP  = 0.012   # rad of joint motion per deg of camera error
PITCH_KP = 0.012

# Integral — corrects slow drift.  Keep small to avoid windup.
ROLL_KI  = 0.001
PITCH_KI = 0.001

# Derivative — damps oscillation.  Increase if the gimbal hunts/rings.
ROLL_KD  = 0.005
PITCH_KD = 0.005

# Integral windup limit (radians)
INTEGRAL_LIMIT = 0.15

# Deadband — ignore camera errors smaller than this (sensor noise floor)
DEADBAND_DEG = 0.5

# Max joint step per control cycle (radians).  Caps speed, prevents jumps.
MAX_STEP_RAD = 0.05   # ~2.9° per cycle at 20 Hz → max ~58°/s

# Joint limits (radians) — stay within physical range
PITCH_LIMIT = 1.20
ROLL_LIMIT  = 1.20

# Control rate and trajectory duration
CONTROL_HZ  = 20.0
TRAJ_STEP   = 1.0 / CONTROL_HZ   # 0.05 s

# Low-pass filter for IMU readings (0 = no filter, 1 = frozen)
BOAT_FILTER_ALPHA   = 0.80   # smooth, boat moves slowly
CAMERA_FILTER_ALPHA = 0.60   # less filtering — we need fast response

# Joint names — order must match your SDF joint order
JOINT_NAMES = ['yaw_joint', 'pitch_joint', 'roll_joint']


# ─────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────

def quat_to_rpy_deg(q):
    """Quaternion → (roll, pitch, yaw) in degrees, ZYX convention."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)

    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)

    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def clamp(v, limit):
    return max(-limit, min(limit, v))


# ─────────────────────────────────────────────────────────────
# NODE
# ─────────────────────────────────────────────────────────────

class GimbalStabilizerNode(Node):

    def __init__(self):
        super().__init__('gimbal_stabilizer_node')

        # ── Boat IMU (display + status only, not used for control) ──────
        self.boat_roll  = 0.0
        self.boat_pitch = 0.0
        self.boat_yaw   = 0.0
        self.boat_imu_ready = False

        # ── Camera IMU (primary feedback for control) ────────────────────
        self.cam_roll  = 0.0
        self.cam_pitch = 0.0
        self.cam_imu_ready = False

        # ── Joint position commands (accumulated each cycle) ─────────────
        self.cmd_yaw   = 0.0   # yaw: uncontrolled, always 0
        self.cmd_pitch = 0.0
        self.cmd_roll  = 0.0

        # ── PID state ────────────────────────────────────────────────────
        self.roll_integral  = 0.0
        self.pitch_integral = 0.0
        self.prev_roll_err  = 0.0
        self.prev_pitch_err = 0.0
        self.prev_time      = time.time()

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Imu, '/imu/data',        self._boat_imu_cb,   10)
        self.create_subscription(Imu, '/camera_imu/data', self._camera_imu_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_traj    = self.create_publisher(JointTrajectory, '/gimbal/joint_trajectory', 10)
        self.pub_corr    = self.create_publisher(Vector3,          '/gimbal/roll_correction',  10)
        self.pub_status  = self.create_publisher(Vector3,          '/debug/stabilizer_status', 10)

        # ── Control loop ─────────────────────────────────────────────────
        self.create_timer(TRAJ_STEP, self._control_loop)

        self.get_logger().info(
            f'✅ GimbalStabilizerNode v2 ready\n'
            f'   Control source : camera IMU → /camera_imu/data\n'
            f'   Goal           : camera roll = 0°, pitch = 0°\n'
            f'   Output         : /gimbal/joint_trajectory\n'
            f'   Kp={ROLL_KP}  Ki={ROLL_KI}  Kd={ROLL_KD}  '
            f'  max_step={math.degrees(MAX_STEP_RAD):.1f}°/cycle'
        )

    # ─────────────────────────────────────────
    # IMU CALLBACKS
    # ─────────────────────────────────────────

    def _boat_imu_cb(self, msg: Imu):
        r, p, y = quat_to_rpy_deg(msg.orientation)
        a = BOAT_FILTER_ALPHA
        self.boat_roll  = a * self.boat_roll  + (1 - a) * r
        self.boat_pitch = a * self.boat_pitch + (1 - a) * p
        self.boat_yaw   = a * self.boat_yaw   + (1 - a) * y
        self.boat_imu_ready = True

    def _camera_imu_cb(self, msg: Imu):
        r, p, y = quat_to_rpy_deg(msg.orientation)
        a = CAMERA_FILTER_ALPHA
        self.cam_roll  = a * self.cam_roll  + (1 - a) * r
        self.cam_pitch = a * self.cam_pitch + (1 - a) * p
        self.cam_imu_ready = True

    # ─────────────────────────────────────────
    # CONTROL LOOP
    # ─────────────────────────────────────────

    def _control_loop(self):
        if not self.cam_imu_ready:
            # Camera IMU not yet received — hold joints at zero and wait
            self._publish_traj(0.0, 0.0)
            return

        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now
        if dt <= 0.001:
            return

        # ── Error: how far is the camera from level? ──────────────────────
        # cam_roll > 0  → camera tilted right → need to roll joint left (negative)
        # cam_pitch > 0 → camera nose up      → need to pitch joint down (negative)
        roll_err  = -self.cam_roll    # deg;  negate: positive error → negative correction
        pitch_err = -self.cam_pitch

        # Apply deadband — don't chase sensor noise
        if abs(roll_err)  < DEADBAND_DEG:
            roll_err  = 0.0
        if abs(pitch_err) < DEADBAND_DEG:
            pitch_err = 0.0

        # ── PID ───────────────────────────────────────────────────────────
        # Integral (anti-windup clamp)
        self.roll_integral  = clamp(
            self.roll_integral  + roll_err  * dt, INTEGRAL_LIMIT)
        self.pitch_integral = clamp(
            self.pitch_integral + pitch_err * dt, INTEGRAL_LIMIT)

        # Derivative
        roll_deriv  = (roll_err  - self.prev_roll_err)  / dt
        pitch_deriv = (pitch_err - self.prev_pitch_err) / dt
        self.prev_roll_err  = roll_err
        self.prev_pitch_err = pitch_err

        # PID output in radians (the step to add to the joint command)
        roll_step  = (ROLL_KP  * math.radians(roll_err)  +
                      ROLL_KI  * math.radians(self.roll_integral)  +
                      ROLL_KD  * math.radians(roll_deriv))
        pitch_step = (PITCH_KP * math.radians(pitch_err) +
                      PITCH_KI * math.radians(self.pitch_integral) +
                      PITCH_KD * math.radians(pitch_deriv))

        # Cap the step size so the gimbal can't jump
        roll_step  = clamp(roll_step,  MAX_STEP_RAD)
        pitch_step = clamp(pitch_step, MAX_STEP_RAD)

        # Accumulate into joint command
        self.cmd_roll  = clamp(self.cmd_roll  + roll_step,  ROLL_LIMIT)
        self.cmd_pitch = clamp(self.cmd_pitch + pitch_step, PITCH_LIMIT)

        # ── Publish ───────────────────────────────────────────────────────
        self._publish_traj(self.cmd_pitch, self.cmd_roll)

        # /gimbal/roll_correction (legacy — kept for dashboard compatibility)
        corr = Vector3()
        corr.x = float(self.boat_roll)
        corr.y = float(self.boat_pitch)
        self.pub_corr.publish(corr)

        # /debug/stabilizer_status
        status = Vector3()
        status.x = float(self.boat_roll)    # boat roll  (deg)
        status.y = float(self.boat_pitch)   # boat pitch (deg)
        status.z = float(self.cam_roll)     # camera roll — watch this approach 0
        self.pub_status.publish(status)

        # Log at ~2 Hz
        self.get_logger().info(
            f'🚢 Boat  roll:{self.boat_roll:+6.2f}°  pitch:{self.boat_pitch:+6.2f}° | '
            f'📷 Cam  roll:{self.cam_roll:+6.2f}°  pitch:{self.cam_pitch:+6.2f}° | '
            f'🔧 Cmd  roll:{math.degrees(self.cmd_roll):+6.2f}°  '
            f'pitch:{math.degrees(self.cmd_pitch):+6.2f}°',
            throttle_duration_sec=0.5
        )

    def _publish_traj(self, pitch_rad: float, roll_rad: float):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        # Order matches JOINT_NAMES: yaw, pitch, roll
        pt.positions = [self.cmd_yaw, pitch_rad, roll_rad]
        pt.time_from_start = Duration(sec=0, nanosec=int(TRAJ_STEP * 1e9))
        traj.points = [pt]
        self.pub_traj.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()