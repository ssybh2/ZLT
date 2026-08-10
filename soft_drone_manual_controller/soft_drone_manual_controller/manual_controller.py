#!/usr/bin/env python3
"""Manual-only DJI RC quadrotor controller for ROS 2.

Control path (unchanged for the original/primary IMU):
DJI sticks -> attitude/rate setpoints -> quaternion outer loop -> rate PID ->
X-frame mixer -> DSHOT.

Dual-IMU / simplified DIMD addition:
- Primary/original IMU: /ecat/sn2883658/app2/read
  Remains the ONLY IMU used by the original attitude loop, rate PID, arming, and
  primary RC/IMU failsafe.
- Secondary/new IMU: /ecat/sn2883658/app1/read
  Adds a separate structural-vibration damping branch inspired by DIMD:
      aligned secondary gyro - primary gyro -> band-pass -> bounded auxiliary torque
  The auxiliary torque is added AFTER the original attitude/rate controller and
  BEFORE the existing motor mixer. If the secondary IMU is stale, unsynchronized,
  or has an invalid payload, the auxiliary torque becomes zero immediately and the
  original controller continues unchanged. DIMD only returns after a healthy hold
  period and then ramps in smoothly.

Both IMUs are mounted with the same physical orientation. Their raw sensor frame is
FLU (x forward, y left, z up). The EXISTING controller conversion is preserved
exactly for BOTH IMUs:
    quaternion [w, x, y, z] -> [w, x, -y, -z]
    vectors    [x, y, z]    -> [x, -y, -z]
which yields the controller/body FRD frame (x forward, y right, z down).

IMPORTANT: the DIMD alignment matrix is an optional calibration applied only AFTER
this existing FLU->FRD conversion. Its default is the identity matrix, so it does
not introduce or redefine any coordinate system.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger


def clamp(value: float, lower: float, upper: float) -> float:
    return float(max(lower, min(upper, value)))


def wrap_pi(angle: float) -> float:
    return float(math.atan2(math.sin(angle), math.cos(angle)))


def quat_normalize(q: Sequence[float]) -> np.ndarray:
    q_array = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(q_array))
    if norm < 1.0e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q_array / norm


def quat_multiply(q1: Sequence[float], q2: Sequence[float]) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_inverse(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array([w, -x, -y, -z], dtype=float)


def quat_to_euler(q: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = quat_normalize(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return quat_normalize(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quaternion_error(q_command: Sequence[float], q_measured: Sequence[float]) -> np.ndarray:
    """Return q_measured^-1 * q_command, matching the existing controller."""
    return quat_multiply(quat_inverse(q_measured), q_command)


def apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) <= deadzone else float(value)


def gravity_tilt_quat_frd(acceleration_frd: Sequence[float]) -> np.ndarray | None:
    """Estimate Roll/Pitch from gravity in the existing FRD body/controller frame.

    Existing convention is preserved:
        x: forward
        y: right
        z: down

    sensor_msgs/Imu.linear_acceleration is treated as accelerometer specific
    force. For a stationary level vehicle in FRD, it points approximately
    along -z. Gravity cannot determine yaw, so yaw is set to zero here.
    """
    accel = np.asarray(acceleration_frd, dtype=float)
    if accel.shape != (3,) or not np.all(np.isfinite(accel)):
        return None

    norm = float(np.linalg.norm(accel))
    if norm < 1.0e-6:
        return None

    ax, ay, az = accel / norm
    roll = math.atan2(float(-ay), float(-az))
    pitch = math.atan2(float(ax), math.sqrt(float(ay * ay + az * az)))
    return euler_to_quat(roll, pitch, 0.0)


@dataclass
class RatePid:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    error_deadband: float

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.previous_measurement = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_measurement = 0.0
        self.initialized = False

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = float(setpoint - measurement)
        if abs(error) < self.error_deadband:
            self.integral = 0.0
            self.previous_measurement = float(measurement)
            self.initialized = True
            return 0.0

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        measurement_rate = 0.0
        if self.initialized and dt > 1.0e-6:
            measurement_rate = (measurement - self.previous_measurement) / dt
        self.previous_measurement = float(measurement)
        self.initialized = True

        return float(self.kp * error + self.ki * self.integral - self.kd * measurement_rate)


@dataclass
class BiquadBandPass:
    """Second-order causal band-pass with 0 dB gain at the center frequency.

    This is used only by the optional structural damping branch. The original
    attitude/rate controller is not filtered or otherwise modified.
    """

    sample_rate_hz: float
    center_frequency_hz: float
    bandwidth_hz: float

    def __post_init__(self) -> None:
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0
        self.configure(
            self.sample_rate_hz,
            self.center_frequency_hz,
            self.bandwidth_hz,
        )

    def configure(
        self, sample_rate_hz: float, center_frequency_hz: float, bandwidth_hz: float
    ) -> None:
        fs = max(1.0, float(sample_rate_hz))
        nyquist = 0.5 * fs
        f0 = clamp(float(center_frequency_hz), 1.0e-3, 0.45 * fs)
        bw = max(1.0e-3, float(bandwidth_hz))
        # Q = f0 / BW. A minimum Q avoids a degenerate coefficient set.
        q = max(0.05, f0 / bw)

        omega = 2.0 * math.pi * f0 / fs
        alpha = math.sin(omega) / (2.0 * q)
        a0 = 1.0 + alpha

        self.b0 = alpha / a0
        self.b1 = 0.0
        self.b2 = -alpha / a0
        self.a1 = (-2.0 * math.cos(omega)) / a0
        self.a2 = (1.0 - alpha) / a0
        self.sample_rate_hz = fs
        self.center_frequency_hz = f0
        self.bandwidth_hz = bw
        self.nyquist_hz = nyquist

    def reset(self) -> None:
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

    def update(self, value: float) -> float:
        x0 = float(value)
        y0 = (
            self.b0 * x0
            + self.b1 * self.x1
            + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2
        )
        self.x2 = self.x1
        self.x1 = x0
        self.y2 = self.y1
        self.y1 = y0
        return float(y0)


class ManualDroneController(Node):
    def __init__(self) -> None:
        super().__init__("soft_drone_manual_controller")
        self._lock = threading.RLock()

        self._declare_parameters()
        self._read_parameters()
        self._load_runtime_message_types()
        self._create_ros_entities()
        self._initialize_state()
        self._initialize_controllers()

        self._control_timer = self.create_timer(1.0 / self.control_frequency_hz, self._control_loop)
        self.get_logger().info(
            "Manual controller started: DJI RC -> quaternion attitude -> rate PID -> DSHOT"
        )
        self.get_logger().info(
            "Dual IMU: PRIMARY/old=%s (base flight control), SECONDARY/new=%s "
            "(DIMD structural branch)"
            % (self.imu_topic, self.secondary_imu_topic)
        )
        self.get_logger().info(
            "DIMD control is %s; f0=%.3f Hz, BW=%.3f Hz, gain=%s, limit=%s"
            % (
                "ENABLED" if self.dimd_enabled else "DISABLED",
                self.dimd_center_frequency_hz,
                self.dimd_bandwidth_hz,
                self.dimd_gain.tolist(),
                self.dimd_torque_limit.tolist(),
            )
        )
        self.get_logger().warn(
            "DRY RUN is %s. Remove propellers before changing signs, mixer, or channel order."
            % ("ON" if self.dry_run else "OFF")
        )

    # ------------------------------------------------------------------
    # Parameters and ROS setup
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            # Current EtherCAT mapping in ZLT:
            # app1 = new IMU, app2 = original IMU, app3 = DJI RC, app4 = DSHOT.
            "rc_topic": "/ecat/sn2883658/app3/read",
            "imu_topic": "/ecat/sn2883658/app2/read",
            "secondary_imu_topic": "/ecat/sn2883658/app1/read",
            "dshot_topic": "/ecat/sn2883658/app4/write",
            "rc_message_type": "custom_msgs/msg/ReadDJIRC",
            "dshot_message_type": "custom_msgs/msg/WriteDSHOT",
            "rc_left_y_field": "left_y",
            "rc_left_x_field": "left_x",
            "rc_right_x_field": "right_x",
            "rc_right_y_field": "right_y",
            "rc_left_switch_field": "left_switch",
            "rc_right_switch_field": "right_switch",
            "dshot_channel_fields": ["channel1", "channel2", "channel3", "channel4"],
            "dry_run": False,
            "control_frequency_hz": 1000.0,
            "data_timeout_s": 0.30,
            "require_low_throttle_to_arm": True,
            "require_lock_cycle_to_arm": True,
            "arm_throttle_max": -0.80,
            "lock_switch_value": 2,
            "unlock_switch_values": [1, 3],
            "dshot_lock_value": 48,
            "dshot_unlock_idle_value": 120,
            "deadzone_roll": 0.03,
            "deadzone_pitch": 0.03,
            "deadzone_yaw": 0.03,
            "deadzone_throttle": 0.05,
            "invert_right_y": True,
            "max_roll_pitch_deg": 30.0,
            "stick_angle_multiplier": 1.0,
            "max_manual_yaw_rate_rad_s": 1.0,
            # Existing trim behavior is unchanged.
            "roll_trim_deg": 0.0,
            "pitch_trim_deg": 0.0,
            # IMPORTANT: preserve the existing FLU -> FRD conversion exactly.
            # Raw IMU: x forward, y left, z up.
            # Controller/body: x forward, y right, z down.
            # The new IMU has the same installation direction, so it uses the
            # exact same conversion; no extra mounting rotation is introduced.
            "quaternion_signs_wxyz": [1.0, 1.0, -1.0, -1.0],
            "gyro_signs_xyz": [1.0, -1.0, -1.0],
            "angle_gain": 0.50,
            "attitude_time_constant_s": 0.09,
            "roll_rate_kp": 0.1215,
            "roll_rate_ki": 0.0,
            "roll_rate_kd": 0.001,
            "pitch_rate_kp": 0.1215,
            "pitch_rate_ki": 0.0,
            "pitch_rate_kd": 0.001,
            "yaw_rate_kp": 0.75,
            "yaw_rate_ki": 0.0,
            "yaw_rate_kd": 0.00048,
            "rate_integral_limit": 0.5,
            "yaw_rate_integral_limit": 0.1,
            "rate_error_deadband_rad_s": 0.009,
            "roll_pitch_torque_limit": 1.3,
            "yaw_torque_limit": 1.8,
            "mixer_matrix_flat": [
                -1.0, 1.0, 1.0,
                1.0, -1.0, 1.0,
                1.0, 1.0, -1.0,
                -1.0, -1.0, -1.0,
            ],
            "dshot_scale": 500.0,
            "yaw_dshot_gain": 0.60,
            "max_motor_spread_pwm": 600.0,
            "motor_smoothing_alpha": 0.18,
            "pwm_min_us": 1000.0,
            "pwm_max_us": 2000.0,
            "dshot_min": 48,
            "dshot_max": 2047,
            # Keep the existing code default. YAML below keeps your current
            # runtime mapping [1, 0, 2, 3].
            "dshot_channel_order": [2, 0, 1, 3],

            # --------------------------------------------------------------
            # Simplified two-IMU DIMD structural damping branch.
            # These parameters DO NOT change either IMU coordinate definition.
            # The secondary-to-primary matrix is applied after both sensors have
            # already undergone the exact same existing FLU -> FRD conversion.
            # Identity is correct when both IMUs use the same physical axis
            # orientation, as in the present aircraft.
            # --------------------------------------------------------------
            "dimd_enabled": False,
            "dimd_secondary_timeout_s": 0.05,
            "dimd_max_pair_skew_s": 0.005,
            # Secondary-IMU health/fallback protection. Any invalid sample causes
            # immediate fallback to primary/app2-only control. Recovery is
            # deliberately slower to avoid rapid DIMD on/off chatter.
            "dimd_health_accel_min_norm_m_s2": 0.5,
            "dimd_health_quaternion_norm_min": 0.5,
            "dimd_health_quaternion_norm_max": 1.5,
            "dimd_health_recovery_hold_s": 0.50,
            "dimd_health_recovery_min_samples": 50,
            "dimd_health_publish_period_s": 0.10,
            "dimd_secondary_to_primary_rotation_matrix_flat": [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
            # Set f0/BW from YOUR aircraft's measured structural spectrum.
            # f0 <= 0 intentionally inhibits auxiliary torque.
            "dimd_center_frequency_hz": 0.0,
            "dimd_bandwidth_hz": 2.0,
            # Signed gains map filtered structural-rate components directly to
            # roll/pitch/yaw auxiliary moments. Start at zero and determine the
            # sign with propellers removed / dry-run first.
            "dimd_gain_xyz": [0.0, 0.0, 0.0],
            "dimd_torque_limit_xyz": [0.02, 0.02, 0.02],
            "dimd_residual_deadband_rad_s": 0.0,
            "dimd_ramp_time_s": 1.0,

            "diagnostics_prefix": "/manual_drone",
            "status_log_period_s": 0.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _p(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _read_parameters(self) -> None:
        self.rc_topic = str(self._p("rc_topic"))
        self.imu_topic = str(self._p("imu_topic"))
        self.secondary_imu_topic = str(self._p("secondary_imu_topic"))
        self.dshot_topic = str(self._p("dshot_topic"))
        self.rc_message_type_name = str(self._p("rc_message_type"))
        self.dshot_message_type_name = str(self._p("dshot_message_type"))

        self.rc_fields = {
            "left_y": str(self._p("rc_left_y_field")),
            "left_x": str(self._p("rc_left_x_field")),
            "right_x": str(self._p("rc_right_x_field")),
            "right_y": str(self._p("rc_right_y_field")),
            "left_switch": str(self._p("rc_left_switch_field")),
            "right_switch": str(self._p("rc_right_switch_field")),
        }
        self.dshot_channel_fields = [str(v) for v in self._p("dshot_channel_fields")]
        if len(self.dshot_channel_fields) != 4:
            raise ValueError("dshot_channel_fields must contain exactly four field names")

        self.dry_run = bool(self._p("dry_run"))
        self.control_frequency_hz = max(1.0, float(self._p("control_frequency_hz")))
        self.nominal_dt = 1.0 / self.control_frequency_hz
        self.data_timeout_s = max(0.01, float(self._p("data_timeout_s")))
        self.require_low_throttle_to_arm = bool(self._p("require_low_throttle_to_arm"))
        self.require_lock_cycle_to_arm = bool(self._p("require_lock_cycle_to_arm"))
        self.arm_throttle_max = float(self._p("arm_throttle_max"))
        self.lock_switch_value = int(self._p("lock_switch_value"))
        self.unlock_switch_values = {int(v) for v in self._p("unlock_switch_values")}
        self.dshot_lock_value = int(self._p("dshot_lock_value"))
        self.dshot_unlock_idle_value = int(self._p("dshot_unlock_idle_value"))

        self.deadzone_roll = float(self._p("deadzone_roll"))
        self.deadzone_pitch = float(self._p("deadzone_pitch"))
        self.deadzone_yaw = float(self._p("deadzone_yaw"))
        self.deadzone_throttle = float(self._p("deadzone_throttle"))
        self.invert_right_y = bool(self._p("invert_right_y"))
        self.max_roll_pitch_rad = math.radians(float(self._p("max_roll_pitch_deg")))
        self.stick_angle_multiplier = float(self._p("stick_angle_multiplier"))
        self.max_manual_yaw_rate = float(self._p("max_manual_yaw_rate_rad_s"))
        self.roll_trim_rad = math.radians(float(self._p("roll_trim_deg")))
        self.pitch_trim_rad = math.radians(float(self._p("pitch_trim_deg")))

        self.quaternion_signs = np.asarray(self._p("quaternion_signs_wxyz"), dtype=float)
        self.gyro_signs = np.asarray(self._p("gyro_signs_xyz"), dtype=float)
        if self.quaternion_signs.shape != (4,) or self.gyro_signs.shape != (3,):
            raise ValueError("quaternion_signs_wxyz must have 4 values and gyro_signs_xyz 3")

        self.angle_gain = float(self._p("angle_gain"))
        self.attitude_time_constant_s = max(1.0e-3, float(self._p("attitude_time_constant_s")))
        self.rate_error_deadband = float(self._p("rate_error_deadband_rad_s"))
        self.roll_pitch_torque_limit = float(self._p("roll_pitch_torque_limit"))
        self.yaw_torque_limit = float(self._p("yaw_torque_limit"))

        mixer_flat = np.asarray(self._p("mixer_matrix_flat"), dtype=float)
        if mixer_flat.size != 12:
            raise ValueError("mixer_matrix_flat must contain 12 numbers")
        self.mixer_matrix = mixer_flat.reshape((4, 3))
        self.dshot_scale = float(self._p("dshot_scale"))
        self.yaw_dshot_gain = float(self._p("yaw_dshot_gain"))
        self.max_motor_spread_pwm = max(0.0, float(self._p("max_motor_spread_pwm")))
        self.motor_smoothing_alpha = clamp(float(self._p("motor_smoothing_alpha")), 0.0, 1.0)
        self.pwm_min_us = float(self._p("pwm_min_us"))
        self.pwm_max_us = float(self._p("pwm_max_us"))
        self.dshot_min = int(self._p("dshot_min"))
        self.dshot_max = int(self._p("dshot_max"))
        self.dshot_channel_order = [int(v) for v in self._p("dshot_channel_order")]
        if sorted(self.dshot_channel_order) != [0, 1, 2, 3]:
            raise ValueError("dshot_channel_order must be a permutation of [0, 1, 2, 3]")

        # Simplified DIMD branch. Existing base-flight parameters above are untouched.
        self.dimd_enabled = bool(self._p("dimd_enabled"))
        self.dimd_secondary_timeout_s = max(
            0.001, float(self._p("dimd_secondary_timeout_s"))
        )
        self.dimd_max_pair_skew_s = max(
            0.0, float(self._p("dimd_max_pair_skew_s"))
        )
        self.dimd_health_accel_min_norm = max(
            0.0, float(self._p("dimd_health_accel_min_norm_m_s2"))
        )
        self.dimd_health_quaternion_norm_min = max(
            0.0, float(self._p("dimd_health_quaternion_norm_min"))
        )
        self.dimd_health_quaternion_norm_max = max(
            self.dimd_health_quaternion_norm_min,
            float(self._p("dimd_health_quaternion_norm_max")),
        )
        self.dimd_health_recovery_hold_s = max(
            0.0, float(self._p("dimd_health_recovery_hold_s"))
        )
        self.dimd_health_recovery_min_samples = max(
            1, int(self._p("dimd_health_recovery_min_samples"))
        )
        self.dimd_health_publish_period_s = max(
            0.02, float(self._p("dimd_health_publish_period_s"))
        )
        dimd_rotation_flat = np.asarray(
            self._p("dimd_secondary_to_primary_rotation_matrix_flat"), dtype=float
        )
        if dimd_rotation_flat.size != 9:
            raise ValueError(
                "dimd_secondary_to_primary_rotation_matrix_flat must contain 9 numbers"
            )
        self.dimd_secondary_to_primary_rotation = dimd_rotation_flat.reshape((3, 3))
        if not np.all(np.isfinite(self.dimd_secondary_to_primary_rotation)):
            raise ValueError("DIMD secondary-to-primary rotation matrix must be finite")

        self.dimd_center_frequency_hz = float(self._p("dimd_center_frequency_hz"))
        self.dimd_bandwidth_hz = max(1.0e-3, float(self._p("dimd_bandwidth_hz")))
        self.dimd_gain = np.asarray(self._p("dimd_gain_xyz"), dtype=float)
        self.dimd_torque_limit = np.asarray(self._p("dimd_torque_limit_xyz"), dtype=float)
        if self.dimd_gain.shape != (3,) or self.dimd_torque_limit.shape != (3,):
            raise ValueError("dimd_gain_xyz and dimd_torque_limit_xyz must each have 3 values")
        self.dimd_torque_limit = np.abs(self.dimd_torque_limit)
        self.dimd_residual_deadband = max(
            0.0, float(self._p("dimd_residual_deadband_rad_s"))
        )
        self.dimd_ramp_time_s = max(0.0, float(self._p("dimd_ramp_time_s")))

        nyquist = 0.5 * self.control_frequency_hz
        if self.dimd_center_frequency_hz > 0.0 and self.dimd_center_frequency_hz >= 0.45 * self.control_frequency_hz:
            raise ValueError(
                "dimd_center_frequency_hz must be below 45% of control_frequency_hz "
                f"(current Nyquist={nyquist:.1f} Hz)"
            )

        self.diagnostics_prefix = str(self._p("diagnostics_prefix")).rstrip("/")
        self.status_log_period_s = max(0.1, float(self._p("status_log_period_s")))

    def _load_runtime_message_types(self) -> None:
        try:
            self.rc_msg_type = get_message(self.rc_message_type_name)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"Cannot load RC type '{self.rc_message_type_name}'. "
                "Build/source the interface package or change rc_message_type."
            ) from exc
        try:
            self.dshot_msg_type = get_message(self.dshot_message_type_name)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"Cannot load DSHOT type '{self.dshot_message_type_name}'. "
                "Build/source the interface package or change dshot_message_type."
            ) from exc

    def _create_ros_entities(self) -> None:
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.dshot_pub = self.create_publisher(self.dshot_msg_type, self.dshot_topic, 10)
        self.rc_sub = self.create_subscription(
            self.rc_msg_type, self.rc_topic, self._rc_callback, qos_best_effort
        )

        # Primary/original IMU. This remains the existing flight-control path.
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_callback, qos_best_effort
        )

        # Secondary/new IMU. It feeds the optional DIMD structural branch, but
        # never replaces the primary IMU and never participates in arming/failsafe.
        self.secondary_imu_sub = self.create_subscription(
            Imu,
            self.secondary_imu_topic,
            self._secondary_imu_callback,
            qos_best_effort,
        )

        # Existing diagnostics: keep names and meaning unchanged (primary IMU).
        self.imu_angle_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/imu_angle_deg", qos_reliable
        )
        self.imu_gyro_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/imu_gyro_rad_s", qos_reliable
        )

        # New observer diagnostics for the second IMU and the IMU pair.
        self.secondary_imu_angle_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/secondary_imu_angle_deg", qos_reliable
        )
        self.secondary_imu_gyro_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/secondary_imu_gyro_rad_s", qos_reliable
        )
        self.imu_common_gyro_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/imu_common_gyro_rad_s", qos_reliable
        )
        self.imu_diff_gyro_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/imu_diff_gyro_rad_s", qos_reliable
        )
        self.imu_relative_angle_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/imu_relative_angle_deg", qos_reliable
        )

        # DIMD diagnostics. Existing diagnostic topic names above are unchanged.
        self.dimd_secondary_aligned_pub = self.create_publisher(
            Vector3,
            f"{self.diagnostics_prefix}/dimd_secondary_gyro_aligned_rad_s",
            qos_reliable,
        )
        self.dimd_residual_raw_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/dimd_residual_raw_rad_s", qos_reliable
        )
        self.dimd_modal_rate_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/dimd_modal_rate_rad_s", qos_reliable
        )
        self.dimd_torque_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/dimd_torque_command", qos_reliable
        )
        self.base_torque_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/torque_base_command", qos_reliable
        )
        self.dimd_active_pub = self.create_publisher(
            Bool, f"{self.diagnostics_prefix}/dimd_active", qos_reliable
        )
        self.dimd_secondary_healthy_pub = self.create_publisher(
            Bool, f"{self.diagnostics_prefix}/dimd_secondary_healthy", qos_reliable
        )
        self.dimd_fallback_pub = self.create_publisher(
            Bool, f"{self.diagnostics_prefix}/dimd_fallback_active", qos_reliable
        )
        self.dimd_health_reason_pub = self.create_publisher(
            String, f"{self.diagnostics_prefix}/dimd_health_reason", qos_reliable
        )

        self.torque_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/torque_command", qos_reliable
        )
        self.motor_pwm_pub = self.create_publisher(
            Float64MultiArray, f"{self.diagnostics_prefix}/motor_pwm_us", qos_reliable
        )
        self.motor_dshot_pub = self.create_publisher(
            Float64MultiArray, f"{self.diagnostics_prefix}/motor_dshot", qos_reliable
        )
        self.armed_pub = self.create_publisher(
            Bool, f"{self.diagnostics_prefix}/armed", qos_reliable
        )
        self.reset_zero_srv = self.create_service(
            Trigger, f"{self.diagnostics_prefix}/reset_attitude_zero", self._reset_zero_callback
        )

    def _initialize_state(self) -> None:
        self.rc_data = {
            "left_y": -1.0,
            "left_x": 0.0,
            "right_x": 0.0,
            "right_y": 0.0,
            "left_switch": self.lock_switch_value,
            "right_switch": 2,
        }
        self.last_rc_time = 0.0
        self.last_imu_time = 0.0
        self.primary_imu_stamp_s = 0.0
        self.last_control_time = 0.0
        self.last_status_log_time = 0.0
        self.last_arm_block_log_time = 0.0

        # ------------------------------------------------------------------
        # Existing primary/original IMU state: names and control use unchanged.
        # ------------------------------------------------------------------
        self.current_abs_quat: np.ndarray | None = None
        self.initial_quat: np.ndarray | None = None
        self.relative_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.relative_euler = np.zeros(3, dtype=float)
        self.gyro = np.zeros(3, dtype=float)
        self.imu_initialized = False

        # ------------------------------------------------------------------
        # New secondary IMU observer state. Same FLU -> FRD conversion.
        # ------------------------------------------------------------------
        self.secondary_current_abs_quat: np.ndarray | None = None
        self.secondary_initial_quat: np.ndarray | None = None
        self.secondary_relative_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.secondary_relative_euler = np.zeros(3, dtype=float)
        self.secondary_gyro = np.zeros(3, dtype=float)
        self.secondary_accel = np.zeros(3, dtype=float)
        self.secondary_imu_initialized = False
        self.last_secondary_imu_time = 0.0
        self.secondary_imu_stamp_s = 0.0

        # Secondary-IMU health state. Invalid app1 data NEVER affects the base
        # controller: it only forces DIMD torque to zero. Recovery requires a
        # continuous healthy window before DIMD is allowed to ramp back in.
        self.secondary_sample_valid = False
        self.dimd_secondary_healthy = False
        self.dimd_secondary_health_reason = "startup"
        self.dimd_secondary_recovery_start_time = 0.0
        self.dimd_secondary_recovery_samples = 0
        self.dimd_secondary_fault_count = 0
        self.last_dimd_health_publish_time = 0.0

        # Pair observer outputs, all expressed in the existing FRD body frame.
        self.imu_common_gyro = np.zeros(3, dtype=float)
        self.imu_diff_gyro = np.zeros(3, dtype=float)
        self.imu_pair_reference_quat: np.ndarray | None = None
        self.imu_relative_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.imu_relative_euler = np.zeros(3, dtype=float)

        # DIMD branch state. This is separate from the original controller state.
        self.dimd_secondary_gyro_aligned = np.zeros(3, dtype=float)
        self.dimd_residual_raw = np.zeros(3, dtype=float)
        self.dimd_modal_rate = np.zeros(3, dtype=float)
        self.dimd_torque = np.zeros(3, dtype=float)
        self.dimd_active = False
        self.dimd_ramp = 0.0

        self.armed = False
        self.arm_permission = not self.require_lock_cycle_to_arm
        self.motor_pwm = np.full(4, self.pwm_min_us, dtype=float)
        self.last_internal_dshot = np.full(4, self.dshot_lock_value, dtype=int)

    def _initialize_controllers(self) -> None:
        common_limit = float(self._p("rate_integral_limit"))
        yaw_limit = float(self._p("yaw_rate_integral_limit"))
        self.roll_rate_pid = RatePid(
            float(self._p("roll_rate_kp")),
            float(self._p("roll_rate_ki")),
            float(self._p("roll_rate_kd")),
            common_limit,
            self.rate_error_deadband,
        )
        self.pitch_rate_pid = RatePid(
            float(self._p("pitch_rate_kp")),
            float(self._p("pitch_rate_ki")),
            float(self._p("pitch_rate_kd")),
            common_limit,
            self.rate_error_deadband,
        )
        self.yaw_rate_pid = RatePid(
            float(self._p("yaw_rate_kp")),
            float(self._p("yaw_rate_ki")),
            float(self._p("yaw_rate_kd")),
            yaw_limit,
            self.rate_error_deadband,
        )

        # One independent second-order band-pass per FRD axis. A placeholder
        # center frequency is used internally when f0<=0 only so the objects can
        # exist; _compute_dimd_torque() inhibits auxiliary torque until a real
        # positive f0 is configured.
        filter_center = (
            self.dimd_center_frequency_hz
            if self.dimd_center_frequency_hz > 0.0
            else 1.0
        )
        self.dimd_filters = [
            BiquadBandPass(
                self.control_frequency_hz,
                filter_center,
                self.dimd_bandwidth_hz,
            )
            for _ in range(3)
        ]

    # ------------------------------------------------------------------
    # Callbacks and safety state
    # ------------------------------------------------------------------
    @staticmethod
    def _read_field(message: Any, field_name: str) -> Any:
        if not hasattr(message, field_name):
            raise AttributeError(
                f"Message type {type(message).__name__} has no field '{field_name}'"
            )
        return getattr(message, field_name)

    def _rc_callback(self, message: Any) -> None:
        with self._lock:
            try:
                self.rc_data["left_y"] = float(self._read_field(message, self.rc_fields["left_y"]))
                self.rc_data["left_x"] = float(self._read_field(message, self.rc_fields["left_x"]))
                self.rc_data["right_x"] = float(self._read_field(message, self.rc_fields["right_x"]))
                right_y = float(self._read_field(message, self.rc_fields["right_y"]))
                self.rc_data["right_y"] = -right_y if self.invert_right_y else right_y
                self.rc_data["left_switch"] = int(
                    self._read_field(message, self.rc_fields["left_switch"])
                )
                self.rc_data["right_switch"] = int(
                    self._read_field(message, self.rc_fields["right_switch"])
                )
            except (AttributeError, TypeError, ValueError) as exc:
                self.get_logger().error(f"Invalid RC message mapping: {exc}")
                return

            self.last_rc_time = self._now_seconds()
            self._update_arming_state()

    def _imu_callback(self, message: Imu) -> None:
        """Original/primary IMU callback.

        This is intentionally kept as the flight-control IMU path. The existing
        FLU -> FRD coordinate conversion and existing initialization convention
        are preserved.
        """
        with self._lock:
            q_raw = np.array(
                [
                    message.orientation.w,
                    message.orientation.x,
                    message.orientation.y,
                    message.orientation.z,
                ],
                dtype=float,
            )

            # EXISTING conversion, unchanged:
            # [w, x, y, z] -> [w, x, -y, -z]
            # IMU FLU (x forward, y left, z up)
            # -> controller/body FRD (x forward, y right, z down).
            q_abs = quat_normalize(q_raw * self.quaternion_signs)
            self.current_abs_quat = q_abs

            if self.initial_quat is None:
                accel_raw = np.array(
                    [
                        message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z,
                    ],
                    dtype=float,
                )

                # EXISTING conversion, unchanged: [x, y, z] -> [x, -y, -z].
                accel_frd = accel_raw * self.gyro_signs
                q_gravity_tilt = gravity_tilt_quat_frd(accel_frd)
                if q_gravity_tilt is None:
                    return

                # EXISTING relative-quaternion convention, unchanged.
                self.initial_quat = quat_normalize(
                    quat_multiply(quat_inverse(q_gravity_tilt), q_abs)
                )
                self.imu_initialized = True
                self.get_logger().info(
                    "Primary IMU initialized in existing FRD convention: "
                    "Roll/Pitch from gravity, Yaw from first valid quaternion"
                )

            # EXISTING order, unchanged: q_relative = q_absolute * inverse(q_reference)
            self.relative_quat = quat_normalize(
                quat_multiply(q_abs, quat_inverse(self.initial_quat))
            )
            self.relative_euler = np.asarray(quat_to_euler(self.relative_quat), dtype=float)
            self.relative_euler[2] = wrap_pi(float(self.relative_euler[2]))

            gyro_raw = np.array(
                [
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ],
                dtype=float,
            )

            # EXISTING conversion, unchanged: [x, y, z] -> [x, -y, -z].
            self.gyro = gyro_raw * self.gyro_signs
            self.last_imu_time = self._now_seconds()
            self.primary_imu_stamp_s = self._message_stamp_seconds(message)
            self._publish_imu_diagnostics()

            # Observer update only. Does not feed anything back into control.
            self._update_dual_imu_observer()

    def _secondary_imu_callback(self, message: Imu) -> None:
        """New IMU callback for diagnostics and optional DIMD damping.

        The new IMU is mounted exactly like the original IMU, so it uses the
        SAME raw-FLU -> controller-FRD conversion. No additional rotation,
        remapping, or coordinate-frame definition is introduced.

        Health protection is fail-fast / recover-slow:
        - one clearly invalid secondary sample immediately disables DIMD;
        - the original app2 controller continues completely unchanged;
        - app1 must remain continuously healthy for the configured recovery
          window before DIMD is allowed to ramp back in.
        """
        with self._lock:
            now = self._now_seconds()
            self.last_secondary_imu_time = now
            self.secondary_imu_stamp_s = self._message_stamp_seconds(message)

            q_raw = np.array(
                [
                    message.orientation.w,
                    message.orientation.x,
                    message.orientation.y,
                    message.orientation.z,
                ],
                dtype=float,
            )
            accel_raw = np.array(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ],
                dtype=float,
            )
            gyro_raw = np.array(
                [
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ],
                dtype=float,
            )

            sample_valid, reason = self._validate_secondary_imu_sample(
                q_raw, accel_raw, gyro_raw
            )
            self.secondary_sample_valid = sample_valid
            self._update_secondary_health_from_sample(now, sample_valid, reason)

            # Never copy an invalid app1 sample into the observer/DIMD state.
            # The last good sample may remain stored, but health=false guarantees
            # _compute_dimd_torque() returns exactly zero.
            if not sample_valid:
                self._publish_dimd_health_diagnostics(now, force=True)
                return

            # SAME existing conversion as the original IMU: FLU -> FRD.
            q_abs = quat_normalize(q_raw * self.quaternion_signs)
            accel_frd = accel_raw * self.gyro_signs
            gyro_frd = gyro_raw * self.gyro_signs

            self.secondary_current_abs_quat = q_abs
            self.secondary_accel = accel_frd
            self.secondary_gyro = gyro_frd

            if self.secondary_initial_quat is None:
                q_gravity_tilt = gravity_tilt_quat_frd(accel_frd)
                if q_gravity_tilt is None:
                    self._set_secondary_health_fault(
                        "gravity_initialization_failed", now
                    )
                    self._publish_dimd_health_diagnostics(now, force=True)
                    return

                # Same initialization convention as the original IMU.
                self.secondary_initial_quat = quat_normalize(
                    quat_multiply(quat_inverse(q_gravity_tilt), q_abs)
                )
                self.secondary_imu_initialized = True
                self.get_logger().info(
                    "Secondary IMU initialized with the same existing FRD conversion "
                    "(DIMD-capable branch)"
                )

            self.secondary_relative_quat = quat_normalize(
                quat_multiply(q_abs, quat_inverse(self.secondary_initial_quat))
            )
            self.secondary_relative_euler = np.asarray(
                quat_to_euler(self.secondary_relative_quat), dtype=float
            )
            self.secondary_relative_euler[2] = wrap_pi(
                float(self.secondary_relative_euler[2])
            )

            self._publish_secondary_imu_diagnostics()
            self._update_dual_imu_observer()
            self._publish_dimd_health_diagnostics(now)

    def _update_dual_imu_observer(self) -> None:
        """Compute observer-only quantities after both IMUs are valid.

        All angular velocities are already expressed in the SAME existing FRD
        body/controller axes before entering this function.

        common = (primary + secondary) / 2
        diff   = secondary - primary

        The diff signal is deliberately the full physical difference, not half
        the difference. For a perfectly anti-phase pair (+A and -A), diff=2A.
        The existing imu_diff diagnostic remains observer-only. The DIMD branch
        uses a separate aligned residual and only affects motors when explicitly
        enabled with a valid f0 and nonzero gains.
        """
        if not self.imu_initialized or not self.secondary_imu_initialized:
            return
        if self.current_abs_quat is None or self.secondary_current_abs_quat is None:
            return

        self.imu_common_gyro = 0.5 * (self.gyro + self.secondary_gyro)
        # EXISTING diagnostic meaning is preserved exactly.
        self.imu_diff_gyro = self.secondary_gyro - self.gyro

        # DIMD uses a separate optional calibration matrix AFTER both IMUs have
        # already been converted with the existing FLU -> FRD signs. The default
        # matrix is identity, so no coordinate-system definition is changed.
        self.dimd_secondary_gyro_aligned = (
            self.dimd_secondary_to_primary_rotation @ self.secondary_gyro
        )
        self.dimd_residual_raw = self.dimd_secondary_gyro_aligned - self.gyro

        # Relative orientation between the two IMU-bearing rods.
        # Both q's have already undergone the SAME FLU -> FRD conversion.
        pair_abs = quat_normalize(
            quat_multiply(
                self.secondary_current_abs_quat,
                quat_inverse(self.current_abs_quat),
            )
        )

        # Remove the fixed initial offset between the two IMUs. This affects
        # only the new diagnostic topic; it does not touch either IMU's control
        # coordinate definition or the primary flight-control zero.
        if self.imu_pair_reference_quat is None:
            self.imu_pair_reference_quat = pair_abs.copy()

        self.imu_relative_quat = quat_normalize(
            quat_multiply(pair_abs, quat_inverse(self.imu_pair_reference_quat))
        )
        self.imu_relative_euler = np.asarray(
            quat_to_euler(self.imu_relative_quat), dtype=float
        )
        self.imu_relative_euler[2] = wrap_pi(float(self.imu_relative_euler[2]))

        self._publish_dual_imu_diagnostics()

    def _validate_secondary_imu_sample(
        self,
        q_raw: np.ndarray,
        accel_raw: np.ndarray,
        gyro_raw: np.ndarray,
    ) -> tuple[bool, str]:
        """Validate one raw app1 message without changing coordinate definitions.

        The observed failure mode is orientation still present while acceleration
        and angular velocity collapse to zeros. Acceleration is the strongest
        health discriminator because a normal grounded/flying vehicle should not
        report a near-zero specific-force vector continuously, whereas zero gyro
        can be perfectly valid when stationary.
        """
        if not (
            np.all(np.isfinite(q_raw))
            and np.all(np.isfinite(accel_raw))
            and np.all(np.isfinite(gyro_raw))
        ):
            return False, "non_finite_secondary_sample"

        q_norm = float(np.linalg.norm(q_raw))
        if not (
            self.dimd_health_quaternion_norm_min
            <= q_norm
            <= self.dimd_health_quaternion_norm_max
        ):
            return False, "secondary_quaternion_norm_invalid"

        accel_norm = float(np.linalg.norm(accel_raw))
        if accel_norm < self.dimd_health_accel_min_norm:
            return False, "secondary_acceleration_missing_or_zero"

        return True, "ok"

    def _set_secondary_health_fault(self, reason: str, now: float) -> None:
        was_healthy = self.dimd_secondary_healthy
        old_reason = self.dimd_secondary_health_reason
        was_recovering = self.dimd_secondary_recovery_start_time > 0.0
        transition = was_healthy or old_reason != reason or was_recovering

        self.dimd_secondary_healthy = False
        self.dimd_secondary_health_reason = str(reason)
        self.dimd_secondary_recovery_start_time = 0.0
        self.dimd_secondary_recovery_samples = 0

        if not transition:
            return

        self.dimd_secondary_fault_count += 1

        # Immediately remove all DIMD memory/torque. Base app2 flight control is
        # untouched and continues in the same control-loop iteration.
        self._reset_dimd_state()

        log = self.get_logger().warn if self.dimd_enabled else self.get_logger().info
        log(
            "DIMD FALLBACK -> PRIMARY/app2 ONLY: secondary IMU unhealthy "
            f"({reason}); DIMD torque forced to zero"
        )
        self._publish_dimd_health_diagnostics(now, force=True)

    def _update_secondary_health_from_sample(
        self, now: float, sample_valid: bool, reason: str
    ) -> None:
        if not sample_valid:
            self._set_secondary_health_fault(reason, now)
            return

        if self.dimd_secondary_healthy:
            self.dimd_secondary_health_reason = "ok"
            return

        if self.dimd_secondary_recovery_start_time <= 0.0:
            self.dimd_secondary_recovery_start_time = now
            self.dimd_secondary_recovery_samples = 1
            self.dimd_secondary_health_reason = "recovering"
            return

        self.dimd_secondary_recovery_samples += 1
        healthy_time = now - self.dimd_secondary_recovery_start_time
        if (
            healthy_time >= self.dimd_health_recovery_hold_s
            and self.dimd_secondary_recovery_samples
            >= self.dimd_health_recovery_min_samples
        ):
            self.dimd_secondary_healthy = True
            self.dimd_secondary_health_reason = "ok"
            self._reset_dimd_state()
            self.get_logger().info(
                "Secondary IMU healthy again after %.3f s / %d valid samples; "
                "DIMD may re-enter with %.3f s ramp if enabled"
                % (
                    healthy_time,
                    self.dimd_secondary_recovery_samples,
                    self.dimd_ramp_time_s,
                )
            )
            self._publish_dimd_health_diagnostics(now, force=True)

    def _publish_dimd_health_diagnostics(
        self, now: float | None = None, *, force: bool = False
    ) -> None:
        if now is None:
            now = self._now_seconds()
        if (
            not force
            and (now - self.last_dimd_health_publish_time)
            < self.dimd_health_publish_period_s
        ):
            return
        self.last_dimd_health_publish_time = now

        healthy_msg = Bool()
        healthy_msg.data = bool(self.dimd_secondary_healthy)
        self.dimd_secondary_healthy_pub.publish(healthy_msg)

        fallback_msg = Bool()
        fallback_msg.data = bool(
            self.dimd_enabled and not self.dimd_secondary_healthy
        )
        self.dimd_fallback_pub.publish(fallback_msg)

        reason_msg = String()
        reason_msg.data = str(self.dimd_secondary_health_reason)
        self.dimd_health_reason_pub.publish(reason_msg)

    def _reset_zero_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        # Existing service behavior remains tied to the original/primary IMU.
        with self._lock:
            if self.armed:
                response.success = False
                response.message = "Refused: disarm before resetting the attitude zero"
                return response
            if self.current_abs_quat is None:
                response.success = False
                response.message = "No IMU quaternion has been received"
                return response
            self.initial_quat = self.current_abs_quat.copy()
            self.relative_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            self.relative_euler[:] = 0.0
            self._reset_controllers()
            response.success = True
            response.message = "Attitude zero reset to the current primary IMU orientation"
            return response

    def _update_arming_state(self) -> None:
        switch = int(self.rc_data["right_switch"])
        if switch == self.lock_switch_value:
            self.arm_permission = True
            if self.armed:
                self._disarm("DJI right switch moved to LOCK")
            return

        if switch not in self.unlock_switch_values or self.armed:
            return
        if self.require_lock_cycle_to_arm and not self.arm_permission:
            self._throttled_warning("Cannot arm: move the DJI right switch to LOCK first")
            return
        if not self.imu_initialized:
            self._throttled_warning("Cannot arm: IMU has not initialized")
            return
        if self.require_low_throttle_to_arm and self.rc_data["left_y"] > self.arm_throttle_max:
            self._throttled_warning(
                "Cannot arm: lower the DJI throttle stick below %.2f (current %.2f)"
                % (self.arm_throttle_max, self.rc_data["left_y"])
            )
            return

        self.armed = True
        self.arm_permission = False
        self._reset_controllers()
        self._reset_dimd_state()
        self.motor_pwm[:] = self.pwm_min_us
        self.get_logger().warn("ARMED in MANUAL mode")
        self._publish_raw_dshot_uniform(self.dshot_unlock_idle_value)
        self._publish_armed()

    def _disarm(self, reason: str) -> None:
        was_armed = self.armed
        self.armed = False
        self._reset_controllers()
        self._reset_dimd_state()
        self.motor_pwm[:] = self.pwm_min_us
        self._publish_raw_dshot_uniform(self.dshot_lock_value)
        self._publish_armed()
        if was_armed:
            self.get_logger().warn(f"DISARMED: {reason}")

    def _check_data_validity(self, now: float) -> bool:
        # Keep the existing safety behavior: RC + PRIMARY/original IMU only.
        # A secondary-IMU timeout must NOT disarm the aircraft in this version.
        rc_ok = self.last_rc_time > 0.0 and (now - self.last_rc_time) <= self.data_timeout_s
        imu_ok = self.last_imu_time > 0.0 and (now - self.last_imu_time) <= self.data_timeout_s
        if rc_ok and imu_ok:
            return True
        if self.armed:
            missing = []
            if not rc_ok:
                missing.append("RC")
            if not imu_ok:
                missing.append("IMU")
            self._disarm("/".join(missing) + " timeout")
        return False

    def _reset_controllers(self) -> None:
        self.roll_rate_pid.reset()
        self.pitch_rate_pid.reset()
        self.yaw_rate_pid.reset()

    def _reset_dimd_state(self) -> None:
        for filter_axis in getattr(self, "dimd_filters", []):
            filter_axis.reset()
        self.dimd_modal_rate[:] = 0.0
        self.dimd_torque[:] = 0.0
        self.dimd_active = False
        self.dimd_ramp = 0.0

    def _dimd_inputs_valid(self, now: float) -> bool:
        if not self.imu_initialized or not self.secondary_imu_initialized:
            return False
        if not self.secondary_sample_valid or not self.dimd_secondary_healthy:
            return False
        if self.last_secondary_imu_time <= 0.0:
            return False
        if (now - self.last_secondary_imu_time) > self.dimd_secondary_timeout_s:
            self._set_secondary_health_fault("secondary_timeout", now)
            return False

        # Source timestamps are used for pair synchronization. A bad pair does
        # not disarm the aircraft; it immediately drops only the DIMD branch.
        if self.primary_imu_stamp_s > 0.0 and self.secondary_imu_stamp_s > 0.0:
            if (
                abs(self.primary_imu_stamp_s - self.secondary_imu_stamp_s)
                > self.dimd_max_pair_skew_s
            ):
                self._set_secondary_health_fault("imu_pair_timestamp_skew", now)
                return False
        return True

    def _compute_dimd_torque(self, now: float, dt: float) -> np.ndarray:
        """Return optional bounded structural damping moment in existing FRD axes.

        Simplified two-IMU DIMD branch:
            secondary gyro (existing FRD)
              -> optional fixed alignment calibration
              -> subtract primary/body-reference gyro
              -> second-order band-pass around measured structural mode
              -> signed per-axis gain
              -> per-axis auxiliary moment saturation

        The original attitude/rate controller never consumes the secondary IMU.
        Any invalid secondary input makes this function return exactly zero.
        """
        self.dimd_torque[:] = 0.0
        self.dimd_active = False

        if not self.dimd_enabled:
            self.dimd_ramp = 0.0
            return self.dimd_torque.copy()
        if self.dimd_center_frequency_hz <= 0.0:
            self.dimd_ramp = 0.0
            return self.dimd_torque.copy()
        if not self._dimd_inputs_valid(now):
            self._reset_dimd_state()
            return self.dimd_torque.copy()

        residual = self.dimd_residual_raw.copy()
        if self.dimd_residual_deadband > 0.0:
            residual = np.array(
                [
                    apply_deadzone(float(v), self.dimd_residual_deadband)
                    for v in residual
                ],
                dtype=float,
            )

        self.dimd_modal_rate = np.array(
            [self.dimd_filters[i].update(float(residual[i])) for i in range(3)],
            dtype=float,
        )

        requested = self.dimd_gain * self.dimd_modal_rate
        requested = np.clip(
            requested, -self.dimd_torque_limit, self.dimd_torque_limit
        )

        if self.dimd_ramp_time_s > 0.0:
            self.dimd_ramp = min(1.0, self.dimd_ramp + dt / self.dimd_ramp_time_s)
        else:
            self.dimd_ramp = 1.0

        self.dimd_torque = requested * self.dimd_ramp
        self.dimd_active = True
        return self.dimd_torque.copy()

    # ------------------------------------------------------------------
    # Manual flight control -- original base controller unchanged
    # ------------------------------------------------------------------
    def _process_sticks(self) -> tuple[float, float, float, float]:
        roll_raw = apply_deadzone(self.rc_data["right_x"], self.deadzone_roll)
        pitch_raw = apply_deadzone(self.rc_data["right_y"], self.deadzone_pitch)
        yaw_raw = apply_deadzone(self.rc_data["left_x"], self.deadzone_yaw)
        throttle_raw = apply_deadzone(self.rc_data["left_y"], self.deadzone_throttle)

        target_limit = self.max_roll_pitch_rad * self.stick_angle_multiplier

        roll_target = self.roll_trim_rad + clamp(roll_raw, -1.0, 1.0) * target_limit
        pitch_target = self.pitch_trim_rad + clamp(pitch_raw, -1.0, 1.0) * target_limit
        yaw_rate_target = clamp(yaw_raw, -1.0, 1.0) * self.max_manual_yaw_rate

        throttle_norm = (clamp(throttle_raw, -1.0, 1.0) + 1.0) / 2.0
        throttle_pwm_increment = throttle_norm * (self.pwm_max_us - self.pwm_min_us)
        return throttle_pwm_increment, roll_target, pitch_target, yaw_rate_target

    def _attitude_and_rate_control(
        self,
        roll_target: float,
        pitch_target: float,
        yaw_rate_target: float,
        dt: float,
    ) -> np.ndarray:
        # IMPORTANT: still uses ONLY the original/primary IMU values.
        roll_measured, pitch_measured, yaw_measured = self.relative_euler
        q_setpoint = euler_to_quat(roll_target, pitch_target, float(yaw_measured))
        q_measured = euler_to_quat(
            float(roll_measured), float(pitch_measured), float(yaw_measured)
        )
        if float(np.dot(q_setpoint, q_measured)) < 0.0:
            q_setpoint = -q_setpoint

        q_error = quaternion_error(q_setpoint, q_measured)
        sign = 1.0 if q_error[0] >= 0.0 else -1.0
        rate_setpoint = (
            sign
            * q_error[1:4]
            * (2.0 / self.attitude_time_constant_s)
            * self.angle_gain
        )
        roll_rate_target = float(rate_setpoint[0])
        pitch_rate_target = float(rate_setpoint[1])

        # IMPORTANT: still uses ONLY self.gyro from the original/app2 IMU.
        torque_roll = self.roll_rate_pid.update(roll_rate_target, float(self.gyro[0]), dt)
        torque_pitch = self.pitch_rate_pid.update(pitch_rate_target, float(self.gyro[1]), dt)
        torque_yaw = self.yaw_rate_pid.update(yaw_rate_target, float(self.gyro[2]), dt)

        return np.array(
            [
                clamp(torque_roll, -self.roll_pitch_torque_limit, self.roll_pitch_torque_limit),
                clamp(torque_pitch, -self.roll_pitch_torque_limit, self.roll_pitch_torque_limit),
                clamp(torque_yaw, -self.yaw_torque_limit, self.yaw_torque_limit),
            ],
            dtype=float,
        )

    def _mix_motors(self, throttle_increment: float, torque: np.ndarray) -> np.ndarray:
        base_pwm = self.pwm_min_us + throttle_increment
        mixer_torque = torque.copy()
        mixer_torque[2] *= self.yaw_dshot_gain
        requested = base_pwm + self.mixer_matrix.dot(mixer_torque) * self.dshot_scale

        spread = float(np.max(requested) - np.min(requested))
        if self.max_motor_spread_pwm > 0.0 and spread > self.max_motor_spread_pwm:
            average = float(np.mean(requested))
            scale = self.max_motor_spread_pwm / spread
            requested = average + (requested - average) * scale

        requested = np.clip(requested, self.pwm_min_us, self.pwm_max_us)
        smoothed = (
            self.motor_smoothing_alpha * requested
            + (1.0 - self.motor_smoothing_alpha) * self.motor_pwm
        )
        return np.clip(smoothed, self.pwm_min_us, self.pwm_max_us)

    def _control_loop(self) -> None:
        with self._lock:
            now = self._now_seconds()
            dt = self._compute_dt(now)
            if not self._check_data_validity(now):
                self._publish_raw_dshot_uniform(self.dshot_lock_value)
                return
            if not self.armed:
                self._publish_raw_dshot_uniform(self.dshot_lock_value)
                return

            throttle, roll_target, pitch_target, yaw_rate_target = self._process_sticks()

            # Existing base controller is unchanged and still uses ONLY app2.
            torque_base = self._attitude_and_rate_control(
                roll_target, pitch_target, yaw_rate_target, dt
            )

            # Simplified DIMD auxiliary structural damping branch from app1.
            # A stale/invalid app1 makes torque_dimd exactly zero; it never
            # disarms or replaces the original app2 control path.
            torque_dimd = self._compute_dimd_torque(now, dt)
            torque = torque_base + torque_dimd
            torque = np.array(
                [
                    clamp(
                        float(torque[0]),
                        -self.roll_pitch_torque_limit,
                        self.roll_pitch_torque_limit,
                    ),
                    clamp(
                        float(torque[1]),
                        -self.roll_pitch_torque_limit,
                        self.roll_pitch_torque_limit,
                    ),
                    clamp(
                        float(torque[2]),
                        -self.yaw_torque_limit,
                        self.yaw_torque_limit,
                    ),
                ],
                dtype=float,
            )

            self.motor_pwm = self._mix_motors(throttle, torque)
            internal_dshot = np.array(
                [self._pwm_to_dshot(v) for v in self.motor_pwm], dtype=int
            )
            self.last_internal_dshot = internal_dshot

            self._publish_motor_command(internal_dshot)
            self._publish_control_diagnostics(
                torque,
                self.motor_pwm,
                internal_dshot,
                torque_base=torque_base,
                torque_dimd=torque_dimd,
            )
            self._log_status(now, roll_target, pitch_target, yaw_rate_target)

    # ------------------------------------------------------------------
    # DSHOT and diagnostics
    # ------------------------------------------------------------------
    def _pwm_to_dshot(self, pwm_us: float) -> int:
        pwm = clamp(pwm_us, self.pwm_min_us, self.pwm_max_us)
        ratio = (pwm - self.pwm_min_us) / max(1.0e-9, self.pwm_max_us - self.pwm_min_us)
        return int(round(self.dshot_min + ratio * (self.dshot_max - self.dshot_min)))

    def _build_dshot_message(self, internal_values: Iterable[int]) -> Any:
        internal = [int(v) for v in internal_values]
        outgoing = [internal[index] for index in self.dshot_channel_order]
        message = self.dshot_msg_type()
        for field_name, value in zip(self.dshot_channel_fields, outgoing):
            if not hasattr(message, field_name):
                raise AttributeError(
                    f"DSHOT message type {self.dshot_message_type_name} has no field '{field_name}'"
                )
            setattr(message, field_name, int(value))
        return message

    def _publish_motor_command(self, internal_dshot: Sequence[int]) -> None:
        if self.dry_run:
            return
        try:
            self.dshot_pub.publish(self._build_dshot_message(internal_dshot))
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Failed to publish DSHOT command: {exc}")
            self._disarm("DSHOT message mapping error")

    def _publish_raw_dshot_uniform(self, raw_value: int) -> None:
        internal = [int(raw_value)] * 4
        self.last_internal_dshot = np.asarray(internal, dtype=int)
        if self.dry_run:
            return
        try:
            self.dshot_pub.publish(self._build_dshot_message(internal))
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Failed to publish raw DSHOT value: {exc}")

    def _publish_imu_diagnostics(self) -> None:
        # Existing primary-IMU diagnostic topics, unchanged.
        angle = Vector3()
        angle.x = math.degrees(float(self.relative_euler[0]))
        angle.y = math.degrees(float(self.relative_euler[1]))
        angle.z = math.degrees(float(self.relative_euler[2]))
        self.imu_angle_pub.publish(angle)

        gyro = Vector3()
        gyro.x = float(self.gyro[0])
        gyro.y = float(self.gyro[1])
        gyro.z = float(self.gyro[2])
        self.imu_gyro_pub.publish(gyro)

    def _publish_secondary_imu_diagnostics(self) -> None:
        angle = Vector3()
        angle.x = math.degrees(float(self.secondary_relative_euler[0]))
        angle.y = math.degrees(float(self.secondary_relative_euler[1]))
        angle.z = math.degrees(float(self.secondary_relative_euler[2]))
        self.secondary_imu_angle_pub.publish(angle)

        gyro = Vector3()
        gyro.x = float(self.secondary_gyro[0])
        gyro.y = float(self.secondary_gyro[1])
        gyro.z = float(self.secondary_gyro[2])
        self.secondary_imu_gyro_pub.publish(gyro)

    def _publish_dual_imu_diagnostics(self) -> None:
        common = Vector3()
        common.x = float(self.imu_common_gyro[0])
        common.y = float(self.imu_common_gyro[1])
        common.z = float(self.imu_common_gyro[2])
        self.imu_common_gyro_pub.publish(common)

        diff = Vector3()
        diff.x = float(self.imu_diff_gyro[0])
        diff.y = float(self.imu_diff_gyro[1])
        diff.z = float(self.imu_diff_gyro[2])
        self.imu_diff_gyro_pub.publish(diff)

        relative_angle = Vector3()
        relative_angle.x = math.degrees(float(self.imu_relative_euler[0]))
        relative_angle.y = math.degrees(float(self.imu_relative_euler[1]))
        relative_angle.z = math.degrees(float(self.imu_relative_euler[2]))
        self.imu_relative_angle_pub.publish(relative_angle)

        aligned = Vector3()
        aligned.x, aligned.y, aligned.z = [
            float(v) for v in self.dimd_secondary_gyro_aligned
        ]
        self.dimd_secondary_aligned_pub.publish(aligned)

        residual = Vector3()
        residual.x, residual.y, residual.z = [float(v) for v in self.dimd_residual_raw]
        self.dimd_residual_raw_pub.publish(residual)

    def _publish_control_diagnostics(
        self,
        torque: np.ndarray,
        pwm: np.ndarray,
        dshot: np.ndarray,
        *,
        torque_base: np.ndarray | None = None,
        torque_dimd: np.ndarray | None = None,
    ) -> None:
        torque_msg = Vector3()
        torque_msg.x, torque_msg.y, torque_msg.z = [float(v) for v in torque]
        self.torque_pub.publish(torque_msg)

        base_values = torque if torque_base is None else torque_base
        base_msg = Vector3()
        base_msg.x, base_msg.y, base_msg.z = [float(v) for v in base_values]
        self.base_torque_pub.publish(base_msg)

        dimd_values = np.zeros(3, dtype=float) if torque_dimd is None else torque_dimd
        dimd_msg = Vector3()
        dimd_msg.x, dimd_msg.y, dimd_msg.z = [float(v) for v in dimd_values]
        self.dimd_torque_pub.publish(dimd_msg)

        modal_msg = Vector3()
        modal_msg.x, modal_msg.y, modal_msg.z = [float(v) for v in self.dimd_modal_rate]
        self.dimd_modal_rate_pub.publish(modal_msg)

        active_msg = Bool()
        active_msg.data = bool(self.dimd_active)
        self.dimd_active_pub.publish(active_msg)
        self._publish_dimd_health_diagnostics()

        pwm_msg = Float64MultiArray()
        pwm_msg.data = [float(v) for v in pwm]
        self.motor_pwm_pub.publish(pwm_msg)

        dshot_msg = Float64MultiArray()
        dshot_msg.data = [float(v) for v in dshot]
        self.motor_dshot_pub.publish(dshot_msg)
        self._publish_armed()

    def _publish_armed(self) -> None:
        message = Bool()
        message.data = bool(self.armed)
        self.armed_pub.publish(message)

    def _log_status(
        self, now: float, roll_target: float, pitch_target: float, yaw_rate_target: float
    ) -> None:
        if (now - self.last_status_log_time) < self.status_log_period_s:
            return
        self.last_status_log_time = now
        roll, pitch, yaw = [math.degrees(float(v)) for v in self.relative_euler]
        self.get_logger().info(
            "MANUAL | RPY=(%.1f, %.1f, %.1f) deg | target RP=(%.1f, %.1f) deg | "
            "yaw rate=%.2f rad/s | PWM=%s | DSHOT=%s | DIMD=%s | SEC=%s%s"
            % (
                roll,
                pitch,
                yaw,
                math.degrees(roll_target),
                math.degrees(pitch_target),
                yaw_rate_target,
                np.rint(self.motor_pwm).astype(int).tolist(),
                self.last_internal_dshot.tolist(),
                "ACTIVE" if self.dimd_active else "OFF",
                "HEALTHY" if self.dimd_secondary_healthy else self.dimd_secondary_health_reason,
                " | DRY_RUN" if self.dry_run else "",
            )
        )

    # ------------------------------------------------------------------
    # Utilities and shutdown
    # ------------------------------------------------------------------
    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def _message_stamp_seconds(message: Imu) -> float:
        try:
            sec = float(message.header.stamp.sec)
            nanosec = float(message.header.stamp.nanosec)
            stamp = sec + nanosec * 1.0e-9
            return stamp if stamp > 0.0 else 0.0
        except (AttributeError, TypeError, ValueError):
            return 0.0

    def _compute_dt(self, now: float) -> float:
        if self.last_control_time <= 0.0:
            dt = self.nominal_dt
        else:
            dt = now - self.last_control_time
            if dt <= 0.0 or dt > max(0.05, 10.0 * self.nominal_dt):
                dt = self.nominal_dt
        self.last_control_time = now
        return float(dt)

    def _throttled_warning(self, text: str) -> None:
        now = time.monotonic()
        if (now - self.last_arm_block_log_time) >= 1.0:
            self.get_logger().warn(text)
            self.last_arm_block_log_time = now

    def safe_shutdown(self) -> None:
        with self._lock:
            self.armed = False
            self._reset_dimd_state()
            # rclpy may already have invalidated the context after Ctrl+C.
            # In that case do not attempt the old final DSHOT publishes, which
            # otherwise raise "publisher's context is invalid" during shutdown.
            if self.dry_run or not rclpy.ok():
                return
            try:
                for _ in range(5):
                    self._publish_raw_dshot_uniform(self.dshot_lock_value)
                    time.sleep(0.01)
            except Exception as exc:  # shutdown must never mask process exit
                self.get_logger().warn(f"Shutdown DSHOT publish skipped: {exc}")


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    controller: ManualDroneController | None = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        controller = ManualDroneController()
        executor.add_node(controller)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if controller is not None:
            controller.safe_shutdown()
            executor.remove_node(controller)
            controller.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
