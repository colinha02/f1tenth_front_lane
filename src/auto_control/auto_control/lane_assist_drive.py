#!/usr/bin/env python3
"""Guarded low-speed lane-assist controller for the front-camera detector."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray


class LaneAssistDrive(Node):
    def __init__(self) -> None:
        super().__init__("lane_assist_drive")
        for name, value in {
            "model_topic": "/front_lane/model",
            "emergency_stop_topic": "/auto/emergency_stop",
            "connection_status_topic": "/vesc/connected",
            "duty_topic": "/vesc/duty",
            "servo_position_topic": "/vesc/servo_position",
            "cruise_duty": 0.025,
            "max_duty": 0.035,
            "servo_left": 0.98,
            "servo_center": 0.46,
            "servo_right": 0.02,
            "lateral_gain": 0.75,
            "lookahead_gain": 0.50,
            "max_steering": 0.35,
            "minimum_confidence": 0.95,
            "required_valid_frames": 10,
            "model_timeout_sec": 0.20,
            "control_rate_hz": 30.0,
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.cruise_duty = min(float(value("cruise_duty")), float(value("max_duty")))
        self.servo_left = float(value("servo_left"))
        self.servo_center = float(value("servo_center"))
        self.servo_right = float(value("servo_right"))
        self.lateral_gain = float(value("lateral_gain"))
        self.lookahead_gain = float(value("lookahead_gain"))
        self.max_steering = abs(float(value("max_steering")))
        self.minimum_confidence = float(value("minimum_confidence"))
        self.required_valid_frames = max(1, int(value("required_valid_frames")))
        self.model_timeout_sec = max(0.05, float(value("model_timeout_sec")))

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.duty_pub = self.create_publisher(Float32, str(value("duty_topic")), command_qos)
        self.servo_pub = self.create_publisher(Float32, str(value("servo_position_topic")), command_qos)
        self.model_sub = self.create_subscription(
            Float32MultiArray, str(value("model_topic")), self._on_model, command_qos
        )
        self.stop_sub = self.create_subscription(
            Bool, str(value("emergency_stop_topic")), self._on_emergency_stop, command_qos
        )
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.connection_sub = self.create_subscription(
            Bool, str(value("connection_status_topic")), self._on_connection, connection_qos
        )

        self.connected = False
        self.emergency_stopped = False
        self.valid_frames = 0
        self.last_model_time = -1.0
        self.lateral_error = 0.0
        self.lookahead_offset = 0.0
        self.model_valid = False
        self.driving = False
        self.timer = self.create_timer(1.0 / max(5.0, float(value("control_rate_hz"))), self._on_timer)
        self.get_logger().warn(
            "Lane assist armed at low duty. Focus the camera preview and press SPACE for emergency stop."
        )

    def _on_connection(self, msg: Bool) -> None:
        self.connected = bool(msg.data)
        if not self.connected:
            self._stop("VESC disconnected")

    def _on_emergency_stop(self, msg: Bool) -> None:
        if msg.data:
            self.emergency_stopped = True
            self._stop("emergency stop")

    def _on_model(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            return
        self.last_model_time = self._now()
        confidence, lateral, lookahead, _, left, right = msg.data[:6]
        self.model_valid = (
            confidence >= self.minimum_confidence and left >= 0.5 and right >= 0.5
        )
        if self.model_valid:
            self.valid_frames += 1
            self.lateral_error = float(lateral)
            self.lookahead_offset = float(lookahead)
        else:
            self.valid_frames = 0

    def _on_timer(self) -> None:
        fresh = self.last_model_time >= 0.0 and self._now() - self.last_model_time <= self.model_timeout_sec
        can_drive = (
            self.connected and not self.emergency_stopped and fresh and self.model_valid
            and self.valid_frames >= self.required_valid_frames
        )
        if not can_drive:
            self._stop(None)
            return
        steering = max(-self.max_steering, min(
            self.max_steering,
            self.lateral_gain * self.lateral_error + self.lookahead_gain * self.lookahead_offset,
        ))
        if steering < 0.0:
            servo = self.servo_center + (self.servo_left - self.servo_center) * (-steering)
        else:
            servo = self.servo_center + (self.servo_right - self.servo_center) * steering
        self.servo_pub.publish(Float32(data=float(servo)))
        self.duty_pub.publish(Float32(data=self.cruise_duty))
        if not self.driving:
            self.driving = True
            self.get_logger().warn("Lane model verified. Low-speed autonomous drive started.")

    def _stop(self, reason: str | None) -> None:
        self.duty_pub.publish(Float32(data=0.0))
        self.servo_pub.publish(Float32(data=self.servo_center))
        if self.driving or reason:
            if reason:
                self.get_logger().warn(f"STOP: {reason}; duty=0.")
            self.driving = False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def stop_actuators(self) -> None:
        self._stop("node shutdown")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaneAssistDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_actuators()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
