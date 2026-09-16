#!/usr/bin/env python3
"""DJI RC -> DShot manual controller for EtherCAT slave sn2031674.

The RC channel semantics intentionally match ZLT:
  throttle = left_y
  yaw      = left_x
  roll     = right_x
  pitch    = -right_y

Arming policy requested by the user:
  right_switch == 3 -> ARM (on transition, low throttle required by default)
  right_switch == 1 -> DISARM immediately
  other switch positions -> DISARM by default (configurable)

Default motor mode is throttle_only: all four DShot channels receive the same
throttle. An optional x_open_loop mixer is provided for bench experiments only;
it has no IMU feedback and is NOT a stabilized flight controller.
"""

from __future__ import annotations

import math
from typing import List

import rclpy
from custom_msgs.msg import ReadDJIRC, WriteDSHOT
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ManualRcDshot(Node):
    def __init__(self) -> None:
        super().__init__("sn2031674_manual_rc")

        self.declare_parameter("rc_topic", "/ecat/sn2031674/app1/read")
        self.declare_parameter("dshot_topic", "/ecat/sn2031674/app4/write")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("rc_timeout_s", 0.30)
        self.declare_parameter("command_log_period_s", 1.0)

        self.declare_parameter("arm_switch_value", 3)
        self.declare_parameter("disarm_switch_value", 1)
        self.declare_parameter("disarm_on_other_switch_positions", True)
        self.declare_parameter("require_low_throttle_to_arm", True)
        self.declare_parameter("arm_throttle_max", -0.90)
        self.declare_parameter("require_arm_edge", True)

        self.declare_parameter("dshot_disarmed", 0)
        self.declare_parameter("dshot_idle", 48)
        self.declare_parameter("dshot_max", 2047)
        self.declare_parameter("throttle_expo", 1.0)

        self.declare_parameter("mode", "throttle_only")
        self.declare_parameter("pitch_inverted", True)
        self.declare_parameter("stick_deadzone", 0.03)
        self.declare_parameter("open_loop_mix_gain", 0.25)
        self.declare_parameter("dshot_channel_order", [0, 1, 2, 3])

        self.rc_topic = str(self.get_parameter("rc_topic").value)
        self.dshot_topic = str(self.get_parameter("dshot_topic").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.rc_timeout_s = max(0.02, float(self.get_parameter("rc_timeout_s").value))
        self.command_log_period_s = max(
            0.1, float(self.get_parameter("command_log_period_s").value)
        )

        self.arm_switch_value = int(self.get_parameter("arm_switch_value").value)
        self.disarm_switch_value = int(self.get_parameter("disarm_switch_value").value)
        self.disarm_on_other_switch_positions = bool(
            self.get_parameter("disarm_on_other_switch_positions").value
        )
        self.require_low_throttle_to_arm = bool(
            self.get_parameter("require_low_throttle_to_arm").value
        )
        self.arm_throttle_max = float(self.get_parameter("arm_throttle_max").value)
        self.require_arm_edge = bool(self.get_parameter("require_arm_edge").value)

        self.dshot_disarmed = int(self.get_parameter("dshot_disarmed").value)
        self.dshot_idle = int(self.get_parameter("dshot_idle").value)
        self.dshot_max = int(self.get_parameter("dshot_max").value)
        self.throttle_expo = max(0.1, float(self.get_parameter("throttle_expo").value))

        self.mode = str(self.get_parameter("mode").value)
        self.pitch_inverted = bool(self.get_parameter("pitch_inverted").value)
        self.stick_deadzone = max(0.0, float(self.get_parameter("stick_deadzone").value))
        self.open_loop_mix_gain = max(0.0, float(self.get_parameter("open_loop_mix_gain").value))
        self.channel_order = [int(v) for v in self.get_parameter("dshot_channel_order").value]

        if self.mode not in ("throttle_only", "x_open_loop"):
            raise ValueError("mode must be 'throttle_only' or 'x_open_loop'")
        if sorted(self.channel_order) != [0, 1, 2, 3]:
            raise ValueError("dshot_channel_order must be a permutation of [0,1,2,3]")
        if not (0 <= self.dshot_disarmed <= 2047):
            raise ValueError("dshot_disarmed must be in [0, 2047]")
        if not (48 <= self.dshot_idle <= self.dshot_max <= 2047):
            raise ValueError("Require 48 <= dshot_idle <= dshot_max <= 2047")

        qos_rc = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        qos_cmd = QoSProfile(depth=1)

        self.dshot_pub = self.create_publisher(WriteDSHOT, self.dshot_topic, qos_cmd)
        self.armed_pub = self.create_publisher(Bool, "~/armed", 10)
        self.axes_pub = self.create_publisher(Float32MultiArray, "~/manual_axes", 10)
        self.rc_sub = self.create_subscription(ReadDJIRC, self.rc_topic, self._rc_callback, qos_rc)

        self.armed = False
        self.rc_online = False
        self.last_rc_time = 0.0
        self.last_right_switch: int | None = None
        self.last_command_log_time = 0.0
        self.last_dshot_command = [self.dshot_disarmed] * 4

        self.throttle = -1.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.right_switch = 0
        self.left_switch = 0

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self.get_logger().info(
            f"RC: {self.rc_topic} -> DShot: {self.dshot_topic}; mode={self.mode}; dry_run={self.dry_run}"
        )
        self.get_logger().info(
            f"ARM right_switch={self.arm_switch_value}, DISARM right_switch={self.disarm_switch_value}"
        )
        self.get_logger().info(
            f"DShot command status will be printed every {self.command_log_period_s:.1f} s"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _deadzone(self, value: float) -> float:
        value = clamp(value, -1.0, 1.0)
        return 0.0 if abs(value) <= self.stick_deadzone else value

    def _rc_callback(self, msg: ReadDJIRC) -> None:
        now = self._now_s()
        self.last_rc_time = now
        self.rc_online = bool(msg.online)

        # Exact ZLT stick semantics.
        self.throttle = clamp(float(msg.left_y), -1.0, 1.0)
        self.yaw = self._deadzone(float(msg.left_x))
        self.roll = self._deadzone(float(msg.right_x))
        raw_pitch = self._deadzone(float(msg.right_y))
        self.pitch = -raw_pitch if self.pitch_inverted else raw_pitch
        self.left_switch = int(msg.left_switch)
        self.right_switch = int(msg.right_switch)

        axes = Float32MultiArray()
        # [throttle, yaw, roll, pitch, left_switch, right_switch, online]
        axes.data = [
            float(self.throttle),
            float(self.yaw),
            float(self.roll),
            float(self.pitch),
            float(self.left_switch),
            float(self.right_switch),
            1.0 if self.rc_online else 0.0,
        ]
        self.axes_pub.publish(axes)

        if not self.rc_online:
            self._disarm("RC offline")
            self.last_right_switch = self.right_switch
            return

        previous = self.last_right_switch
        self.last_right_switch = self.right_switch

        if self.right_switch == self.disarm_switch_value:
            self._disarm("right switch = DISARM")
            return

        if self.right_switch != self.arm_switch_value:
            if self.disarm_on_other_switch_positions:
                self._disarm("right switch not in ARM position")
            return

        arm_edge = previous is not None and previous != self.arm_switch_value
        if self.require_arm_edge and not arm_edge:
            return
        self._try_arm()

    def _try_arm(self) -> None:
        if self.armed:
            return
        if not self.rc_online:
            self.get_logger().warning("ARM rejected: RC offline")
            return
        if self.require_low_throttle_to_arm and self.throttle > self.arm_throttle_max:
            self.get_logger().warning(
                f"ARM rejected: throttle={self.throttle:.3f}, must be <= {self.arm_throttle_max:.3f}"
            )
            return
        self.armed = True
        self.get_logger().warning("ARMED: manual RC DShot enabled")
        self._publish_armed()

    def _disarm(self, reason: str) -> None:
        was_armed = self.armed
        self.armed = False
        self._publish_dshot([self.dshot_disarmed] * 4)
        self._publish_armed()
        if was_armed:
            self.get_logger().warning(f"DISARMED: {reason}")

    def _publish_armed(self) -> None:
        msg = Bool()
        msg.data = bool(self.armed)
        self.armed_pub.publish(msg)

    def _throttle_norm(self) -> float:
        normalized = (clamp(self.throttle, -1.0, 1.0) + 1.0) * 0.5
        return math.pow(clamp(normalized, 0.0, 1.0), self.throttle_expo)

    def _norm_to_dshot(self, normalized: float) -> int:
        normalized = clamp(normalized, 0.0, 1.0)
        return int(round(self.dshot_idle + normalized * (self.dshot_max - self.dshot_idle)))

    def _compute_motor_values(self) -> List[int]:
        base = self._throttle_norm()
        if self.mode == "throttle_only":
            value = self._norm_to_dshot(base)
            return [value, value, value, value]

        # Optional X-frame open-loop mixer. This intentionally mirrors the axis
        # semantics used by ZLT, but has NO attitude/rate feedback.
        gain = self.open_loop_mix_gain
        r, p, y = self.roll, self.pitch, self.yaw
        mixed = [
            base + gain * (-r + p + y),
            base + gain * ( r - p + y),
            base + gain * ( r + p - y),
            base + gain * (-r - p - y),
        ]
        return [self._norm_to_dshot(v) for v in mixed]

    def _maybe_log_command(self, now: float) -> None:
        if (
            self.last_command_log_time > 0.0
            and (now - self.last_command_log_time) < self.command_log_period_s
        ):
            return
        self.last_command_log_time = now
        suffix = " (DRY-RUN, not published)" if self.dry_run else ""
        c1, c2, c3, c4 = self.last_dshot_command
        self.get_logger().info(
            "RC->DSHOT | online=%d armed=%d right_switch=%d | "
            "throttle=%+.3f yaw=%+.3f roll=%+.3f pitch=%+.3f | "
            "cmd=[%d, %d, %d, %d]%s"
            % (
                1 if self.rc_online else 0,
                1 if self.armed else 0,
                self.right_switch,
                self.throttle,
                self.yaw,
                self.roll,
                self.pitch,
                c1,
                c2,
                c3,
                c4,
                suffix,
            )
        )

    def _control_loop(self) -> None:
        now = self._now_s()
        stale = self.last_rc_time <= 0.0 or (now - self.last_rc_time) > self.rc_timeout_s
        if stale:
            if self.armed:
                self._disarm("RC timeout")
            else:
                self._publish_dshot([self.dshot_disarmed] * 4)
            self._maybe_log_command(now)
            return

        if not self.rc_online or not self.armed:
            self._publish_dshot([self.dshot_disarmed] * 4)
            self._maybe_log_command(now)
            return

        self._publish_dshot(self._compute_motor_values())
        self._publish_armed()
        self._maybe_log_command(now)

    def _publish_dshot(self, internal_values: List[int]) -> None:
        outgoing = [int(internal_values[i]) for i in self.channel_order]
        self.last_dshot_command = outgoing.copy()
        if self.dry_run:
            return
        msg = WriteDSHOT()
        msg.channel1 = outgoing[0]
        msg.channel2 = outgoing[1]
        msg.channel3 = outgoing[2]
        msg.channel4 = outgoing[3]
        self.dshot_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualRcDshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # ros2 launch may shut down the context before rclpy.spin() returns.
        # Re-raise genuine runtime errors that happen while ROS is still active.
        if rclpy.ok():
            raise
    finally:
        # Best-effort zero command before shutdown.
        try:
            node.armed = False
            node._publish_dshot([node.dshot_disarmed] * 4)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
