#!/usr/bin/env python3

"""Straight-drive test node with a terminal space-bar emergency stop.

This node is intentionally independent from the joystick controller.  It only
publishes a centred steering command and a fixed forward ERPM command after a
VESC connection has been verified.  Pressing the space bar in the *terminal
that started this node* latches the vehicle in the stopped state.
"""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32


class StraightRunKeyboardNode(Node):
    def __init__(self) -> None:
        super().__init__("straight_run_keyboard")

        self.declare_parameter("servo_position_topic", "/vesc/servo_position")
        self.declare_parameter("erpm_topic", "/vesc/erpm")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("straight_servo_position", 0.46)
        self.declare_parameter("target_erpm", 2500)
        self.declare_parameter("command_rate_hz", 30.0)
        self.declare_parameter("connection_timeout_sec", 10.0)

        servo_topic = str(self.get_parameter("servo_position_topic").value)
        erpm_topic = str(self.get_parameter("erpm_topic").value)
        connection_topic = str(
            self.get_parameter("connection_status_topic").value
        )
        self.straight_servo_position = float(
            self.get_parameter("straight_servo_position").value
        )
        self.target_erpm = int(self.get_parameter("target_erpm").value)
        command_rate_hz = max(
            5.0, float(self.get_parameter("command_rate_hz").value)
        )
        self.connection_timeout_sec = max(
            1.0, float(self.get_parameter("connection_timeout_sec").value)
        )

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.servo_pub = self.create_publisher(Float32, servo_topic, command_qos)
        self.erpm_pub = self.create_publisher(Int32, erpm_topic, command_qos)

        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.connection_sub = self.create_subscription(
            Bool, connection_topic, self._on_connection_status, connection_qos
        )

        self._vesc_connected = False
        self._connection_received = False
        self._stopped = False
        self._started = False
        self._started_at = self._now_sec()
        self._stop_event = threading.Event()
        self._keyboard_thread: threading.Thread | None = None

        self._timer = self.create_timer(1.0 / command_rate_hz, self._on_timer)
        self._start_keyboard_listener()
        self.get_logger().warn(
            "Straight-drive mode armed: keep the vehicle clear of people. "
            "It will drive straight only after VESC connection is verified."
        )
        self.get_logger().warn(
            "Press SPACE in this terminal to stop immediately. Ctrl+C also stops."
        )

    def _on_connection_status(self, msg: Bool) -> None:
        self._connection_received = True
        self._vesc_connected = bool(msg.data)
        if self._started and not self._vesc_connected:
            self._latch_stop("VESC connection lost")

    def _on_timer(self) -> None:
        if self._stop_event.is_set() and not self._stopped:
            self._latch_stop("space bar pressed")

        if self._stopped:
            self._publish_stop()
            return

        if not self._vesc_connected:
            self._publish_stop()
            if self._now_sec() - self._started_at >= self.connection_timeout_sec:
                status = "disconnected" if self._connection_received else "not received"
                self._latch_stop(f"VESC connection status {status} after timeout")
            return

        if (
            self.erpm_pub.get_subscription_count() < 1
            or self.servo_pub.get_subscription_count() < 1
        ):
            self._publish_stop()
            return

        if not self._started:
            self._started = True
            self.get_logger().info(
                "VESC ready. Driving straight at "
                f"{self.target_erpm} ERPM; SPACE stops the vehicle."
            )

        self.servo_pub.publish(Float32(data=self.straight_servo_position))
        self.erpm_pub.publish(Int32(data=self.target_erpm))

    def _latch_stop(self, reason: str) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._publish_stop()
        self.get_logger().warn(f"STOPPED: {reason}. ERPM 0 is being published.")

    def _publish_stop(self) -> None:
        self.erpm_pub.publish(Int32(data=0))
        self.servo_pub.publish(Float32(data=self.straight_servo_position))

    def _start_keyboard_listener(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "No interactive terminal detected; space-bar stop is unavailable. "
                "Use Ctrl+C to stop."
            )
            return
        self._keyboard_thread = threading.Thread(
            target=self._wait_for_space,
            name="space_bar_stop_listener",
            daemon=True,
        )
        self._keyboard_thread.start()

    def _wait_for_space(self) -> None:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and not self._stop_event.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable and sys.stdin.read(1) == " ":
                    self._stop_event.set()
                    return
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def stop_actuators(self) -> None:
        self._stopped = True
        self._publish_stop()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StraightRunKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Stopping vehicle.")
    finally:
        node.stop_actuators()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
