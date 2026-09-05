#!/usr/bin/env python3
"""Six-IMU extension for the existing ZLT manual flight controller.

Mapping used by the current EtherCAT ProductCode 0x06 configuration:

    /imu/can1/slot1  -> primary / pseudo-body IMU (old app2)
    /imu/can1/slot2  -> rod IMU 0 (old app1 / legacy DIMD secondary)
    /imu/can1/slot3  -> rod IMU 1
    /imu/can2/slot1  -> rod IMU 2
    /imu/can2/slot2  -> rod IMU 3
    /imu/can2/slot3  -> rod IMU 4
    /dji_rc           -> DJI RC read (old app3 semantic role)
    /dshot            -> DShot write (old app4 semantic role)

The original attitude/rate/arming/failsafe controller remains tied only to the
primary IMU.  The five rod IMUs are structural observers.  Rod 0 preserves the
legacy secondary-IMU path; the other four are added without changing the base
flight-control path.

A configurable 3 x 15 projection maps five 3-axis residuals into the existing
three-axis DIMD filter/gain path.  The safe default selects only rod 0 (identity),
which reproduces the old two-IMU DIMD signal.  Set a different projection only
after the five-rod mounting/sign/mode map has been identified from data.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from .manual_controller import ManualDroneController, apply_deadzone


class SixImuManualDroneController(ManualDroneController):
    """Keep the proven base controller and extend only the structural IMU branch."""

    _OLD_RC_TOPIC = "/ecat/sn2883658/app3/read"
    _OLD_PRIMARY_TOPIC = "/ecat/sn2883658/app2/read"
    _OLD_SECONDARY_TOPIC = "/ecat/sn2883658/app1/read"
    _OLD_DSHOT_TOPIC = "/ecat/sn2883658/app4/write"

    _NEW_RC_TOPIC = "/dji_rc"
    _NEW_PRIMARY_TOPIC = "/imu/can1/slot1"
    _NEW_SECONDARY_TOPIC = "/imu/can1/slot2"
    _NEW_DSHOT_TOPIC = "/dshot"

    def _declare_parameters(self) -> None:
        super()._declare_parameters()

        self.declare_parameter(
            "additional_rod_imu_topics",
            [
                "/imu/can1/slot3",
                "/imu/can2/slot1",
                "/imu/can2/slot2",
                "/imu/can2/slot3",
            ],
        )

        # Rotation matrices are applied after the existing FLU -> FRD conversion.
        # Row 0 continues to use dimd_secondary_to_primary_rotation_matrix_flat.
        # These four matrices are for rods 1..4 and default to identity.
        self.declare_parameter(
            "additional_rod_to_primary_rotation_matrices_flat",
            [
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
            ],
        )

        # Flattening order is:
        # [rod0 xyz, rod1 xyz, rod2 xyz, rod3 xyz, rod4 xyz].
        # Default = identity selection of rod0, so existing tuning is unchanged.
        projection = np.zeros((3, 15), dtype=float)
        projection[0, 0] = 1.0
        projection[1, 1] = 1.0
        projection[2, 2] = 1.0
        self.declare_parameter("dimd_rod_projection_matrix_flat", projection.reshape(-1).tolist())
        self.declare_parameter("dimd_projection_nonzero_epsilon", 1.0e-9)

    def _read_parameters(self) -> None:
        super()._read_parameters()

        # Preserve YAML/CLI overrides. Only translate exact historical defaults.
        if self.rc_topic == self._OLD_RC_TOPIC:
            self.rc_topic = self._NEW_RC_TOPIC
        if self.imu_topic == self._OLD_PRIMARY_TOPIC:
            self.imu_topic = self._NEW_PRIMARY_TOPIC
        if self.secondary_imu_topic == self._OLD_SECONDARY_TOPIC:
            self.secondary_imu_topic = self._NEW_SECONDARY_TOPIC
        if self.dshot_topic == self._OLD_DSHOT_TOPIC:
            self.dshot_topic = self._NEW_DSHOT_TOPIC

        self.additional_rod_imu_topics = [
            str(v) for v in self._p("additional_rod_imu_topics")
        ]
        if len(self.additional_rod_imu_topics) != 4:
            raise ValueError("additional_rod_imu_topics must contain exactly four topics")
        if len(set(self.additional_rod_imu_topics)) != 4:
            raise ValueError("additional_rod_imu_topics must be unique")
        if self.secondary_imu_topic in self.additional_rod_imu_topics:
            raise ValueError("secondary_imu_topic must not be duplicated in additional_rod_imu_topics")
        if self.imu_topic in [self.secondary_imu_topic, *self.additional_rod_imu_topics]:
            raise ValueError("primary imu_topic must be different from all rod IMU topics")

        extra_rotation_flat = np.asarray(
            self._p("additional_rod_to_primary_rotation_matrices_flat"), dtype=float
        )
        if extra_rotation_flat.size != 36 or not np.all(np.isfinite(extra_rotation_flat)):
            raise ValueError(
                "additional_rod_to_primary_rotation_matrices_flat must contain 36 finite numbers"
            )
        self.additional_rod_rotations = extra_rotation_flat.reshape((4, 3, 3))

        projection_flat = np.asarray(self._p("dimd_rod_projection_matrix_flat"), dtype=float)
        if projection_flat.size != 45 or not np.all(np.isfinite(projection_flat)):
            raise ValueError("dimd_rod_projection_matrix_flat must contain 45 finite numbers")
        self.dimd_rod_projection = projection_flat.reshape((3, 15))
        self.dimd_projection_nonzero_epsilon = max(
            0.0, float(self._p("dimd_projection_nonzero_epsilon"))
        )

        self.rod_imu_topics = [self.secondary_imu_topic, *self.additional_rod_imu_topics]
        self.required_dimd_rods = np.array(
            [
                bool(
                    np.any(
                        np.abs(self.dimd_rod_projection[:, 3 * i : 3 * i + 3])
                        > self.dimd_projection_nonzero_epsilon
                    )
                )
                for i in range(5)
            ],
            dtype=bool,
        )

    def _create_ros_entities(self) -> None:
        super()._create_ros_entities()

        qos_latest_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_diag = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Rod 0 is the existing secondary subscription created by the base class.
        # Add only rods 1..4 here.
        self.additional_rod_imu_subs = []
        for extra_index, topic in enumerate(self.additional_rod_imu_topics, start=1):
            sub = self.create_subscription(
                Imu,
                topic,
                lambda msg, rod_index=extra_index: self._additional_rod_imu_callback(
                    rod_index, msg
                ),
                qos_latest_best_effort,
            )
            self.additional_rod_imu_subs.append(sub)

        self.rod_aligned_gyro_pubs = []
        self.rod_residual_pubs = []
        self.rod_healthy_pubs = []
        for i in range(5):
            self.rod_aligned_gyro_pubs.append(
                self.create_publisher(
                    Vector3,
                    f"{self.diagnostics_prefix}/dimd/rod{i}/gyro_aligned_rad_s",
                    qos_diag,
                )
            )
            self.rod_residual_pubs.append(
                self.create_publisher(
                    Vector3,
                    f"{self.diagnostics_prefix}/dimd/rod{i}/residual_rad_s",
                    qos_diag,
                )
            )
            self.rod_healthy_pubs.append(
                self.create_publisher(
                    Bool,
                    f"{self.diagnostics_prefix}/dimd/rod{i}/healthy",
                    qos_reliable,
                )
            )

    def _initialize_state(self) -> None:
        super()._initialize_state()

        self.multi_primary_history: list[tuple[float, np.ndarray]] = []
        self.multi_primary_history_limit = 32

        self.rod_gyro_aligned = np.zeros((5, 3), dtype=float)
        self.rod_residual = np.zeros((5, 3), dtype=float)
        self.rod_last_pair_wall_time = np.zeros(5, dtype=float)
        self.rod_last_pair_skew_s = np.zeros(5, dtype=float)
        self.rod_last_diag_publish_time = np.zeros(5, dtype=float)

        # Rod 0 mirrors the existing secondary-IMU health machine. The four
        # additional rods use the same fail-fast/recover-slow thresholds.
        self.extra_rod_sample_valid = np.zeros(4, dtype=bool)
        self.extra_rod_healthy = np.zeros(4, dtype=bool)
        self.extra_rod_health_reason = ["startup"] * 4
        self.extra_rod_recovery_start_time = np.zeros(4, dtype=float)
        self.extra_rod_recovery_samples = np.zeros(4, dtype=int)

    def _imu_callback(self, message: Imu) -> None:
        # Keep every line of the original primary flight-control callback.
        super()._imu_callback(message)

        # Add a shared primary sample history for matching rods 1..4. The legacy
        # rod0/app1 pairing remains handled by the original code path.
        with self._lock:
            stamp = float(self.primary_imu_stamp_s)
            if stamp <= 0.0 or not math.isfinite(stamp):
                return
            self.multi_primary_history.append((stamp, self.gyro.copy()))
            if len(self.multi_primary_history) > self.multi_primary_history_limit:
                del self.multi_primary_history[
                    0 : len(self.multi_primary_history) - self.multi_primary_history_limit
                ]

    def _secondary_imu_callback(self, message: Imu) -> None:
        # Rod0 is exactly the historical app1 IMU, now /imu/can1/slot2.
        super()._secondary_imu_callback(message)
        with self._lock:
            self._mirror_legacy_rod0()
            self._publish_rod_diagnostic(0)

    def _mirror_legacy_rod0(self) -> None:
        self.rod_gyro_aligned[0, :] = self.dimd_secondary_gyro_aligned
        self.rod_residual[0, :] = self.dimd_residual_raw
        self.rod_last_pair_wall_time[0] = float(self.last_dimd_pair_wall_time)
        self.rod_last_pair_skew_s[0] = float(self.last_dimd_pair_skew_s)

    def _additional_rod_imu_callback(self, rod_index: int, message: Imu) -> None:
        """Receive rod 1..4; these sensors never participate in base flight safety."""
        if rod_index < 1 or rod_index > 4:
            return
        extra_index = rod_index - 1

        with self._lock:
            now = self._now_seconds()
            stamp = self._message_stamp_seconds(message)

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
            self.extra_rod_sample_valid[extra_index] = bool(sample_valid)
            self._update_extra_rod_health(extra_index, now, sample_valid, reason)
            if not sample_valid:
                self._publish_rod_diagnostic(rod_index, force=True)
                return

            # Preserve the exact existing raw vector convention: FLU -> FRD.
            gyro_frd = gyro_raw * self.gyro_signs
            matched = self._match_primary_for_extra_rod(stamp)
            if matched is None:
                self.extra_rod_health_reason[extra_index] = "waiting_for_primary_pair"
                self._publish_rod_diagnostic(rod_index)
                return

            primary_stamp, primary_gyro = matched
            aligned = self.additional_rod_rotations[extra_index] @ gyro_frd
            self.rod_gyro_aligned[rod_index, :] = aligned
            self.rod_residual[rod_index, :] = aligned - primary_gyro
            self.rod_last_pair_wall_time[rod_index] = now
            self.rod_last_pair_skew_s[rod_index] = abs(stamp - primary_stamp)
            self._publish_rod_diagnostic(rod_index)

    def _match_primary_for_extra_rod(
        self, rod_stamp_s: float
    ) -> tuple[float, np.ndarray] | None:
        stamp = float(rod_stamp_s)
        if stamp <= 0.0 or not math.isfinite(stamp) or not self.multi_primary_history:
            return None

        best = min(self.multi_primary_history, key=lambda item: abs(item[0] - stamp))
        if abs(best[0] - stamp) > self.dimd_max_pair_skew_s:
            return None
        return float(best[0]), best[1].copy()

    def _update_extra_rod_health(
        self, extra_index: int, now: float, sample_valid: bool, reason: str
    ) -> None:
        if not sample_valid:
            self.extra_rod_healthy[extra_index] = False
            self.extra_rod_health_reason[extra_index] = str(reason)
            self.extra_rod_recovery_start_time[extra_index] = 0.0
            self.extra_rod_recovery_samples[extra_index] = 0
            return

        if self.extra_rod_healthy[extra_index]:
            self.extra_rod_health_reason[extra_index] = "ok"
            return

        if self.extra_rod_recovery_start_time[extra_index] <= 0.0:
            self.extra_rod_recovery_start_time[extra_index] = now
            self.extra_rod_recovery_samples[extra_index] = 1
            self.extra_rod_health_reason[extra_index] = "recovering"
            return

        self.extra_rod_recovery_samples[extra_index] += 1
        healthy_time = now - self.extra_rod_recovery_start_time[extra_index]
        if (
            healthy_time >= self.dimd_health_recovery_hold_s
            and self.extra_rod_recovery_samples[extra_index]
            >= self.dimd_health_recovery_min_samples
        ):
            self.extra_rod_healthy[extra_index] = True
            self.extra_rod_health_reason[extra_index] = "ok"

    def _rod_is_healthy_and_fresh(self, rod_index: int, now: float) -> tuple[bool, str]:
        if rod_index == 0:
            self._mirror_legacy_rod0()
            if not self.secondary_sample_valid or not self.dimd_secondary_healthy:
                return False, "rod0_unhealthy"
        else:
            extra_index = rod_index - 1
            if (
                not self.extra_rod_sample_valid[extra_index]
                or not self.extra_rod_healthy[extra_index]
            ):
                return False, f"rod{rod_index}_unhealthy"

        last_pair = float(self.rod_last_pair_wall_time[rod_index])
        if last_pair <= 0.0:
            return False, f"rod{rod_index}_waiting_for_pair"
        if (now - last_pair) > self.dimd_data_stale_timeout_s:
            return False, f"rod{rod_index}_pair_stale"
        return True, "ok"

    def _compute_dimd_torque(self, now: float, dt: float) -> np.ndarray:
        """Project five rod residuals, then reuse the existing DIMD filter/control path."""
        self.dimd_torque[:] = 0.0
        self.dimd_active = False

        if not self.dimd_enabled or self.dimd_center_frequency_hz <= 0.0:
            self.dimd_ramp = 0.0
            return self.dimd_torque.copy()
        if not self.imu_initialized:
            self.dimd_ramp = 0.0
            self.dimd_data_fresh = False
            self.dimd_inhibit_reason = "primary_imu_not_initialized"
            return self.dimd_torque.copy()
        if not np.any(self.required_dimd_rods):
            self.dimd_ramp = 0.0
            self.dimd_data_fresh = False
            self.dimd_inhibit_reason = "projection_has_no_active_rod"
            return self.dimd_torque.copy()

        self._mirror_legacy_rod0()
        for rod_index, required in enumerate(self.required_dimd_rods.tolist()):
            if not required:
                continue
            ok, reason = self._rod_is_healthy_and_fresh(rod_index, now)
            if not ok:
                self.dimd_torque[:] = 0.0
                self.dimd_active = False
                self.dimd_ramp = 0.0
                self.dimd_data_fresh = False
                self.dimd_inhibit_reason = reason
                return self.dimd_torque.copy()

        self.dimd_data_fresh = True
        self.dimd_inhibit_reason = "ok"

        residual_flat = self.rod_residual.reshape(15)
        residual = self.dimd_rod_projection @ residual_flat
        if self.dimd_residual_deadband > 0.0:
            residual = np.array(
                [
                    apply_deadzone(float(v), self.dimd_residual_deadband)
                    for v in residual
                ],
                dtype=float,
            )

        self.dimd_modal_rate = np.array(
            [
                self.dimd_filters[i].update(float(residual[i]), dt)
                for i in range(3)
            ],
            dtype=float,
        )
        quadrature_lag = np.array(
            [filter_axis.quadrature_lag_output() for filter_axis in self.dimd_filters],
            dtype=float,
        )
        self.dimd_control_rate = (
            self.dimd_phase_cos * self.dimd_modal_rate
            - self.dimd_phase_sin * quadrature_lag
        )

        requested = np.clip(
            self.dimd_gain * self.dimd_control_rate,
            -self.dimd_torque_limit,
            self.dimd_torque_limit,
        )
        if self.dimd_ramp_time_s > 0.0:
            self.dimd_ramp = min(1.0, self.dimd_ramp + dt / self.dimd_ramp_time_s)
        else:
            self.dimd_ramp = 1.0

        self.dimd_torque = requested * self.dimd_ramp
        self.dimd_active = True
        return self.dimd_torque.copy()

    def _publish_rod_diagnostic(self, rod_index: int, *, force: bool = False) -> None:
        now = self._now_seconds()
        if (
            not force
            and self.rod_last_diag_publish_time[rod_index] > 0.0
            and (now - self.rod_last_diag_publish_time[rod_index])
            < self.imu_diagnostics_period_s
        ):
            return
        self.rod_last_diag_publish_time[rod_index] = now

        aligned = Vector3()
        aligned.x, aligned.y, aligned.z = [
            float(v) for v in self.rod_gyro_aligned[rod_index]
        ]
        self.rod_aligned_gyro_pubs[rod_index].publish(aligned)

        residual = Vector3()
        residual.x, residual.y, residual.z = [
            float(v) for v in self.rod_residual[rod_index]
        ]
        self.rod_residual_pubs[rod_index].publish(residual)

        healthy = Bool()
        if rod_index == 0:
            healthy.data = bool(self.secondary_sample_valid and self.dimd_secondary_healthy)
        else:
            extra_index = rod_index - 1
            healthy.data = bool(
                self.extra_rod_sample_valid[extra_index]
                and self.extra_rod_healthy[extra_index]
            )
        self.rod_healthy_pubs[rod_index].publish(healthy)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    controller: SixImuManualDroneController | None = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        controller = SixImuManualDroneController()
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
