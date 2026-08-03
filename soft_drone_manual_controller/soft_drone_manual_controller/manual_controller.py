#!/usr/bin/env python3
"""Manual-only DJI RC quadrotor controller for ROS 2.

The control path is extracted from ssybh2/soft_drone_controller:
DJI sticks -> attitude/rate setpoints -> quaternion outer loop -> rate PID ->
X-frame mixer -> DSHOT.
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
from std_msgs.msg import Bool, Float64MultiArray
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
    """Return q_measured^-1 * q_command, matching the source controller."""
    return quat_multiply(quat_inverse(q_measured), q_command)


def apply_deadzone(value: float, deadzone: float) -> float:
    return 0.0 if abs(value) <= deadzone else float(value)


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
            # Keep derivative state synchronized to avoid a spike when leaving the deadband.
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

        # The source controller uses a negative derivative-on-measurement damping term.
        return float(self.kp * error + self.ki * self.integral - self.kd * measurement_rate)


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
        self.get_logger().warn(
            "DRY RUN is %s. Remove propellers before changing signs, mixer, or channel order."
            % ("ON" if self.dry_run else "OFF")
        )

    # ------------------------------------------------------------------
    # Parameters and ROS setup
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            "rc_topic": "/ecat/sn2293823/app2/read",
            "imu_topic": "/ecat/sn2293823/app1/read",
            "dshot_topic": "/ecat/sn2293823/app3/write",
            "rc_message_type": "custom_msgs/msg/ReadDJIRC",
            "dshot_message_type": "custom_msgs/msg/WriteDSHOT",
            "rc_left_y_field": "left_y",
            "rc_left_x_field": "left_x",
            "rc_right_x_field": "right_x",
            "rc_right_y_field": "right_y",
            "rc_left_switch_field": "left_switch",
            "rc_right_switch_field": "right_switch",
            "dshot_channel_fields": ["channel1", "channel2", "channel3", "channel4"],
            "dry_run": True,
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
            "dshot_channel_order": [2, 0, 1, 3],
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

        self.diagnostics_prefix = str(self._p("diagnostics_prefix")).rstrip("/")
        self.status_log_period_s = max(0.1, float(self._p("status_log_period_s")))

    def _load_runtime_message_types(self) -> None:
        try:
            self.rc_msg_type = get_message(self.rc_message_type_name)
        except Exception as exc:  # noqa: BLE001 - provide actionable ROS error
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
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self._imu_callback, qos_best_effort)

        self.imu_angle_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/imu_angle_deg", qos_reliable
        )
        self.imu_gyro_pub = self.create_publisher(
            Vector3, f"{self.diagnostics_prefix}/imu_gyro_rad_s", qos_reliable
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
        self.last_control_time = 0.0
        self.last_status_log_time = 0.0
        self.last_arm_block_log_time = 0.0

        self.current_abs_quat: np.ndarray | None = None
        self.initial_quat: np.ndarray | None = None
        self.relative_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.relative_euler = np.zeros(3, dtype=float)
        self.gyro = np.zeros(3, dtype=float)
        self.imu_initialized = False
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
                # Right switch is retained for diagnostics only; manual-only mode ignores it.
                self.rc_data["right_switch"] = int(
                    self._read_field(message, self.rc_fields["right_switch"])
                )
            except (AttributeError, TypeError, ValueError) as exc:
                self.get_logger().error(f"Invalid RC message mapping: {exc}")
                return

            self.last_rc_time = self._now_seconds()
            self._update_arming_state()

    def _imu_callback(self, message: Imu) -> None:
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
            q_abs = quat_normalize(q_raw * self.quaternion_signs)
            self.current_abs_quat = q_abs

            if self.initial_quat is None:
                self.initial_quat = q_abs.copy()
                self.imu_initialized = True
                self.get_logger().info("IMU attitude zero initialized from the first valid quaternion")

            # Preserve the relative-quaternion order used by the reference controller.
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
            self.gyro = gyro_raw * self.gyro_signs
            self.last_imu_time = self._now_seconds()
            self._publish_imu_diagnostics()

    def _reset_zero_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
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
            response.message = "Attitude zero reset to the current IMU orientation"
            return response

    def _update_arming_state(self) -> None:
        switch = int(self.rc_data["left_switch"])
        if switch == self.lock_switch_value:
            self.arm_permission = True
            if self.armed:
                self._disarm("DJI left switch moved to LOCK")
            return

        if switch not in self.unlock_switch_values or self.armed:
            return
        if self.require_lock_cycle_to_arm and not self.arm_permission:
            self._throttled_warning("Cannot arm: move the DJI left switch to LOCK first")
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
        self.motor_pwm[:] = self.pwm_min_us
        self.get_logger().warn("ARMED in MANUAL mode")
        self._publish_raw_dshot_uniform(self.dshot_unlock_idle_value)
        self._publish_armed()

    def _disarm(self, reason: str) -> None:
        was_armed = self.armed
        self.armed = False
        self._reset_controllers()
        self.motor_pwm[:] = self.pwm_min_us
        self._publish_raw_dshot_uniform(self.dshot_lock_value)
        self._publish_armed()
        if was_armed:
            self.get_logger().warn(f"DISARMED: {reason}")

    def _check_data_validity(self, now: float) -> bool:
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

    # ------------------------------------------------------------------
    # Manual flight control
    # ------------------------------------------------------------------
    def _process_sticks(self) -> tuple[float, float, float, float]:
        roll_raw = apply_deadzone(self.rc_data["right_x"], self.deadzone_roll)
        pitch_raw = apply_deadzone(self.rc_data["right_y"], self.deadzone_pitch)
        yaw_raw = apply_deadzone(self.rc_data["left_x"], self.deadzone_yaw)
        throttle_raw = apply_deadzone(self.rc_data["left_y"], self.deadzone_throttle)

        target_limit = self.max_roll_pitch_rad * self.stick_angle_multiplier
        roll_target = clamp(roll_raw, -1.0, 1.0) * target_limit
        pitch_target = clamp(pitch_raw, -1.0, 1.0) * target_limit
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
            torque = self._attitude_and_rate_control(
                roll_target, pitch_target, yaw_rate_target, dt
            )
            self.motor_pwm = self._mix_motors(throttle, torque)
            internal_dshot = np.array([self._pwm_to_dshot(v) for v in self.motor_pwm], dtype=int)
            self.last_internal_dshot = internal_dshot

            self._publish_motor_command(internal_dshot)
            self._publish_control_diagnostics(torque, self.motor_pwm, internal_dshot)
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

    def _publish_control_diagnostics(
        self, torque: np.ndarray, pwm: np.ndarray, dshot: np.ndarray
    ) -> None:
        torque_msg = Vector3()
        torque_msg.x, torque_msg.y, torque_msg.z = [float(v) for v in torque]
        self.torque_pub.publish(torque_msg)

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
            "yaw rate=%.2f rad/s | PWM=%s | DSHOT=%s%s"
            % (
                roll,
                pitch,
                yaw,
                math.degrees(roll_target),
                math.degrees(pitch_target),
                yaw_rate_target,
                np.rint(self.motor_pwm).astype(int).tolist(),
                self.last_internal_dshot.tolist(),
                " | DRY_RUN" if self.dry_run else "",
            )
        )

    # ------------------------------------------------------------------
    # Utilities and shutdown
    # ------------------------------------------------------------------
    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

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
            if not self.dry_run:
                # Repeat the lock command to improve the chance it reaches the EtherCAT writer.
                for _ in range(5):
                    self._publish_raw_dshot_uniform(self.dshot_lock_value)
                    time.sleep(0.01)


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
