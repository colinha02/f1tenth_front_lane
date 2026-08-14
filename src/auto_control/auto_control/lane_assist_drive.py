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
            # Intentionally conservative first autonomous-drive values.
            "cruise_duty": 0.040,
            "virtual_cruise_duty": 0.025,
            "max_duty": 0.050,
            "servo_left": 0.98,
            "servo_center": 0.46,
            "servo_right": 0.02,
            "close_gain": 0.35,
            "far_gain": 0.70,
            "max_steering": 0.28,
            "steering_duty_reduction": 0.45,
            "minimum_center_points": 6,
            "required_valid_frames": 10,
            "model_timeout_sec": 0.20,
            "control_rate_hz": 30.0,
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        max_duty = abs(float(value("max_duty")))
        self.cruise_duty = min(abs(float(value("cruise_duty"))), max_duty)
        self.virtual_cruise_duty = min(abs(float(value("virtual_cruise_duty"))), max_duty)
        self.servo_left = float(value("servo_left"))
        self.servo_center = float(value("servo_center"))
        self.servo_right = float(value("servo_right"))
        self.close_gain = float(value("close_gain"))
        self.far_gain = float(value("far_gain"))
        self.max_steering = abs(float(value("max_steering")))
        self.steering_duty_reduction = min(
            0.95, max(0.0, float(value("steering_duty_reduction")))
        )
        self.minimum_center_points = max(2, int(value("minimum_center_points")))
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
        self.close_error = 0.0
        self.far_error = 0.0
        self.virtual_only = False
        self.model_valid = False
        self.driving = False
        self.timer = self.create_timer(1.0 / max(5.0, float(value("control_rate_hz"))), self._on_timer)
        self.get_logger().warn(
            "Lane assist armed at low duty. It starts only after stable CLOSE/FAR targets; focus preview and press SPACE to stop."
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
        valid, close_error, far_error, virtual_only, center_count, _ = msg.data[:6]
        self.model_valid = valid >= 0.5 and center_count >= self.minimum_center_points
        if self.model_valid:
            self.valid_frames += 1
            self.close_error = float(close_error)
            self.far_error = float(far_error)
            self.virtual_only = virtual_only >= 0.5
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
            self.close_gain * self.close_error + self.far_gain * self.far_error,
        ))
        if steering < 0.0:
            servo = self.servo_center + (self.servo_left - self.servo_center) * (-steering)
        else:
            servo = self.servo_center + (self.servo_right - self.servo_center) * steering
        base_duty = self.virtual_cruise_duty if self.virtual_only else self.cruise_duty
        steering_ratio = abs(steering) / max(self.max_steering, 1e-6)
        duty = base_duty * (1.0 - self.steering_duty_reduction * steering_ratio)
        self.servo_pub.publish(Float32(data=float(servo)))
        self.duty_pub.publish(Float32(data=float(max(0.0, duty))))
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
