#!/usr/bin/env python3
"""Direct front-camera lane detector.

The detector deliberately works in image coordinates.  It uses the camera
driver's rectified NV12 stream but does not calculate or consume a BEV image.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


@dataclass
class FitState:
    coefficients: Optional[np.ndarray] = None
    held_frames: int = 0


class FrontLaneDetector(Node):
    """Tracks left and right lane markings with image-space sliding windows."""

    def __init__(self) -> None:
        super().__init__("front_lane_detector")
        self._declare_parameters()
        self._read_parameters()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._mask_pub = self.create_publisher(Image, self.mask_topic, qos)
        self._overlay_pub = self.create_publisher(Image, self.overlay_topic, qos)
        self._model_pub = self.create_publisher(Float32MultiArray, self.model_topic, qos)
        self._image_sub = self.create_subscription(
            Image, self.image_topic, self._on_image, qos
        )

        self._left_state = FitState()
        self._right_state = FitState()
        self._width_fit: Optional[np.ndarray] = None
        self.get_logger().info(
            "Front lane detector started: direct image sliding-window mode "
            "(no BEV, no steering command output)."
        )

    def _declare_parameters(self) -> None:
        for name, value in {
            "image_topic": "/camera/image_rect",
            "mask_topic": "/front_lane/mask",
            "overlay_topic": "/front_lane/overlay",
            "model_topic": "/front_lane/model",
            "roi_top_ratio": 0.34,
            "roi_bottom_ratio": 0.98,
            "window_count": 12,
            "window_margin_px": 85,
            "minimum_window_pixels": 35,
            "minimum_fit_pixels": 180,
            "white_lightness_min": 165,
            "white_saturation_max": 120,
            "morphology_kernel_px": 5,
            "temporal_alpha": 0.70,
            "temporal_maximum_jump_px": 95.0,
            "temporal_hold_frames": 3,
            "expected_lane_width_px": 360.0,
            "minimum_lane_width_px": 180.0,
            "maximum_lane_width_px": 650.0,
            "lookahead_ratio": 0.48,
        }.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        parameter = lambda name: self.get_parameter(name).value
        self.image_topic = str(parameter("image_topic"))
        self.mask_topic = str(parameter("mask_topic"))
        self.overlay_topic = str(parameter("overlay_topic"))
        self.model_topic = str(parameter("model_topic"))
        self.roi_top_ratio = float(parameter("roi_top_ratio"))
        self.roi_bottom_ratio = float(parameter("roi_bottom_ratio"))
        self.window_count = max(4, int(parameter("window_count")))
        self.window_margin_px = max(10, int(parameter("window_margin_px")))
        self.minimum_window_pixels = max(5, int(parameter("minimum_window_pixels")))
        self.minimum_fit_pixels = max(30, int(parameter("minimum_fit_pixels")))
        self.white_lightness_min = int(parameter("white_lightness_min"))
        self.white_saturation_max = int(parameter("white_saturation_max"))
        self.morphology_kernel_px = max(1, int(parameter("morphology_kernel_px")))
        if self.morphology_kernel_px % 2 == 0:
            self.morphology_kernel_px += 1
        self.temporal_alpha = float(parameter("temporal_alpha"))
        self.temporal_maximum_jump_px = float(parameter("temporal_maximum_jump_px"))
        self.temporal_hold_frames = max(0, int(parameter("temporal_hold_frames")))
        self.expected_lane_width_px = float(parameter("expected_lane_width_px"))
        self.minimum_lane_width_px = float(parameter("minimum_lane_width_px"))
        self.maximum_lane_width_px = float(parameter("maximum_lane_width_px"))
        self.lookahead_ratio = float(parameter("lookahead_ratio"))

    @staticmethod
    def _nv12_to_bgr(message: Image) -> np.ndarray:
        if message.encoding.lower() != "nv12":
            raise ValueError(f"expected nv12 image, received '{message.encoding}'")
        stride = max(message.width, message.step)
        rows = message.height * 3 // 2
        expected_size = stride * rows
        if len(message.data) < expected_size:
            raise ValueError("NV12 image data is shorter than height * 3/2 * step")
        nv12 = np.frombuffer(message.data, dtype=np.uint8, count=expected_size)
        nv12 = nv12.reshape((rows, stride))[:, : message.width]
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

    def _candidate_mask(self, bgr: np.ndarray, top: int, bottom: int) -> np.ndarray:
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
        lower = np.array([0, self.white_lightness_min, 0], dtype=np.uint8)
        upper = np.array([179, 255, self.white_saturation_max], dtype=np.uint8)
        mask = cv2.inRange(hls, lower, upper)
        roi_mask = np.zeros_like(mask)
        roi_mask[top:bottom, :] = 255
        mask = cv2.bitwise_and(mask, roi_mask)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.morphology_kernel_px, self.morphology_kernel_px),
        )
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _sliding_window_points(
        self,
        mask: np.ndarray,
        start_x: Optional[int],
        top: int,
        bottom: int,
    ) -> np.ndarray:
        if start_x is None:
            return np.empty((0, 2), dtype=np.float64)
        nonzero_y, nonzero_x = mask.nonzero()
        current_x = int(np.clip(start_x, 0, mask.shape[1] - 1))
        height = max(1, (bottom - top) // self.window_count)
        selected: list[np.ndarray] = []
        for index in range(self.window_count):
            y_high = bottom - index * height
            y_low = max(top, y_high - height)
            in_window = (
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (nonzero_x >= current_x - self.window_margin_px)
                & (nonzero_x <= current_x + self.window_margin_px)
            )
            point_indices = np.flatnonzero(in_window)
            if point_indices.size:
                selected.append(point_indices)
            if point_indices.size >= self.minimum_window_pixels:
                current_x = int(np.mean(nonzero_x[point_indices]))
        if not selected:
            return np.empty((0, 2), dtype=np.float64)
        indices = np.concatenate(selected)
        # Stored as [y, x], matching numpy.polyfit's independent variable.
        return np.column_stack((nonzero_y[indices], nonzero_x[indices])).astype(np.float64)

    @staticmethod
    def _fit(points: np.ndarray, minimum_pixels: int) -> Optional[np.ndarray]:
        if points.shape[0] < minimum_pixels:
            return None
        return np.polyfit(points[:, 0], points[:, 1], 2)

    def _initial_seed(
        self,
        histogram: np.ndarray,
        left: bool,
        previous: Optional[np.ndarray],
        bottom: int,
    ) -> Optional[int]:
        midpoint = histogram.size // 2
        section = histogram[:midpoint] if left else histogram[midpoint:]
        offset = 0 if left else midpoint
        if section.size and int(section.max()) > 0:
            return int(np.argmax(section)) + offset
        if previous is not None:
            return int(np.polyval(previous, bottom - 1))
        return None

    def _accept_fit(
        self, candidate: Optional[np.ndarray], state: FitState, sample_y: np.ndarray
    ) -> tuple[Optional[np.ndarray], bool]:
        if candidate is not None:
            if state.coefficients is None:
                state.coefficients = candidate
                state.held_frames = 0
                return candidate, True
            jump = float(np.median(np.abs(
                np.polyval(candidate, sample_y) - np.polyval(state.coefficients, sample_y)
            )))
            if jump <= self.temporal_maximum_jump_px:
                state.coefficients = (
                    self.temporal_alpha * candidate
                    + (1.0 - self.temporal_alpha) * state.coefficients
                )
                state.held_frames = 0
                return state.coefficients, True
        if state.coefficients is not None and state.held_frames < self.temporal_hold_frames:
            state.held_frames += 1
            return state.coefficients, False
        state.coefficients = None
        state.held_frames = 0
        return None, False

    @staticmethod
    def _points_for_fit(fit: Optional[np.ndarray], ys: np.ndarray, width: int) -> Optional[np.ndarray]:
        if fit is None:
            return None
        xs = np.polyval(fit, ys)
        inside = (xs >= 0) & (xs < width)
        if int(np.count_nonzero(inside)) < 2:
            return None
        return np.column_stack((xs[inside], ys[inside])).astype(np.int32)

    def _publish_image(self, source: Image, image: np.ndarray, encoding: str) -> None:
        message = Image()
        message.header = source.header
        message.height, message.width = image.shape[:2]
        message.encoding = encoding
        message.is_bigendian = False
        message.step = image.strides[0]
        message.data = image.tobytes()
        (self._mask_pub if encoding == "mono8" else self._overlay_pub).publish(message)

    def _on_image(self, message: Image) -> None:
        try:
            bgr = self._nv12_to_bgr(message)
            height, width = bgr.shape[:2]
            top = int(np.clip(self.roi_top_ratio * height, 0, height - 2))
            bottom = int(np.clip(self.roi_bottom_ratio * height, top + 2, height))
            mask = self._candidate_mask(bgr, top, bottom)
            histogram = np.sum(mask[max(top, bottom - 80):bottom] > 0, axis=0)
            sample_y = np.linspace(bottom - 1, top, 8)
            left_seed = self._initial_seed(
                histogram, True, self._left_state.coefficients, bottom
            )
            right_seed = self._initial_seed(
                histogram, False, self._right_state.coefficients, bottom
            )
            left_points = self._sliding_window_points(mask, left_seed, top, bottom)
            right_points = self._sliding_window_points(mask, right_seed, top, bottom)
            left_fit, left_detected = self._accept_fit(
                self._fit(left_points, self.minimum_fit_pixels), self._left_state, sample_y
            )
            right_fit, right_detected = self._accept_fit(
                self._fit(right_points, self.minimum_fit_pixels), self._right_state, sample_y
            )

            if left_detected and right_detected and left_fit is not None and right_fit is not None:
                width_fit = right_fit - left_fit
                lane_widths = np.polyval(width_fit, sample_y)
                if np.all((lane_widths >= self.minimum_lane_width_px) & (lane_widths <= self.maximum_lane_width_px)):
                    self._width_fit = width_fit

            width_fit = self._width_fit
            if width_fit is None:
                width_fit = np.array([0.0, 0.0, self.expected_lane_width_px])
            if left_fit is None and right_fit is not None:
                left_fit = right_fit - width_fit
            elif right_fit is None and left_fit is not None:
                right_fit = left_fit + width_fit

            overlay = bgr.copy()
            ys = np.linspace(bottom - 1, top, 80)
            left_curve = self._points_for_fit(left_fit, ys, width)
            right_curve = self._points_for_fit(right_fit, ys, width)
            if left_curve is not None:
                cv2.polylines(overlay, [left_curve], False, (255, 0, 0), 5)
            if right_curve is not None:
                cv2.polylines(overlay, [right_curve], False, (255, 0, 0), 5)

            valid_sides = int(left_fit is not None) + int(right_fit is not None)
            confidence = 0.0
            lateral_error = 0.0
            lookahead_offset = 0.0
            curvature = 0.0
            if left_fit is not None and right_fit is not None:
                center_fit = 0.5 * (left_fit + right_fit)
                center_curve = self._points_for_fit(center_fit, ys, width)
                if center_curve is not None:
                    cv2.polylines(overlay, [center_curve], False, (0, 255, 0), 3)
                lookahead_y = top + self.lookahead_ratio * (bottom - top)
                bottom_x = float(np.polyval(center_fit, bottom - 1))
                lookahead_x = float(np.polyval(center_fit, lookahead_y))
                lateral_error = (bottom_x - 0.5 * width) / (0.5 * width)
                lookahead_offset = (lookahead_x - 0.5 * width) / (0.5 * width)
                a, b, _ = center_fit
                slope = 2.0 * a * lookahead_y + b
                curvature = float(2.0 * a / ((1.0 + slope * slope) ** 1.5))
                actual_sides = int(left_detected) + int(right_detected)
                confidence = 1.0 if actual_sides == 2 else 0.60

            self._publish_image(message, mask, "mono8")
            self._publish_image(message, overlay, "bgr8")
            self._model_pub.publish(Float32MultiArray(data=[
                float(confidence), float(lateral_error), float(lookahead_offset),
                float(curvature), float(left_detected), float(right_detected),
            ]))
        except Exception as error:
            self.get_logger().error(f"Front lane frame rejected: {error}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontLaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
