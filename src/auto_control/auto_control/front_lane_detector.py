#!/usr/bin/env python3
"""Simple front-camera white-lane tracker (no BEV, no colour model)."""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray


class FrontLaneDetector(Node):
    """Threshold lower-half grayscale pixels and follow them with windows."""

    def __init__(self) -> None:
        super().__init__("front_lane_detector")
        for name, value in {
            "image_topic": "/camera/image_rect",
            "mask_topic": "/front_lane/mask",
            "overlay_topic": "/front_lane/overlay",
            "model_topic": "/front_lane/model",
            "preview_enabled": True,
            "preview_window_name": "Front lane detection",
            "preview_fps": 20.0,
            "processing_fps": 20.0,
            "publish_debug_images": False,
            "roi_top_ratio": 0.5,
            "white_threshold": 180,
            "seed_row_ratio": 0.75,
            "seed_band_height_px": 24,
            "window_count": 20,
            "window_margin_px": 90,
            "minimum_window_pixels": 20,
            "maximum_missing_windows": 2,
            "maximum_component_pixels": 700,
            "maximum_component_width_px": 70,
            "show_diagnostic_windows": True,
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.image_topic = str(value("image_topic"))
        self.mask_topic = str(value("mask_topic"))
        self.overlay_topic = str(value("overlay_topic"))
        self.model_topic = str(value("model_topic"))
        self.preview_enabled = bool(value("preview_enabled"))
        self.preview_window_name = str(value("preview_window_name"))
        self.preview_fps = max(1.0, float(value("preview_fps")))
        self.processing_fps = max(1.0, float(value("processing_fps")))
        self.publish_debug_images = bool(value("publish_debug_images"))
        self.roi_top_ratio = float(value("roi_top_ratio"))
        self.white_threshold = int(value("white_threshold"))
        self.seed_row_ratio = float(value("seed_row_ratio"))
        self.seed_band_height_px = max(3, int(value("seed_band_height_px")))
        self.window_count = max(4, int(value("window_count")))
        self.window_margin_px = max(10, int(value("window_margin_px")))
        self.minimum_window_pixels = max(3, int(value("minimum_window_pixels")))
        self.maximum_missing_windows = max(0, int(value("maximum_missing_windows")))
        self.maximum_component_pixels = max(
            self.minimum_window_pixels, int(value("maximum_component_pixels"))
        )
        self.maximum_component_width_px = max(
            5, int(value("maximum_component_width_px"))
        )
        self.show_diagnostic_windows = bool(value("show_diagnostic_windows"))

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.mask_pub = self.create_publisher(Image, self.mask_topic, qos)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, qos)
        self.model_pub = self.create_publisher(Float32MultiArray, self.model_topic, qos)
        self.stop_pub = self.create_publisher(Bool, "/auto/emergency_stop", qos)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._on_image, qos)
        self.next_process_at = 0.0
        self.next_preview_at = 0.0
        self.get_logger().info(
            "Simple lane tracker started: lower-half grayscale threshold + sliding windows."
        )

    @staticmethod
    def _nv12_to_bgr(message: Image) -> np.ndarray:
        if message.encoding.lower() != "nv12":
            raise ValueError(f"expected nv12 image, received '{message.encoding}'")
        stride = max(message.width, message.step)
        rows = message.height * 3 // 2
        data = np.frombuffer(message.data, dtype=np.uint8, count=stride * rows)
        return cv2.cvtColor(
            data.reshape((rows, stride))[:, : message.width], cv2.COLOR_YUV2BGR_NV12
        )

    def _binary_white_mask(self, gray: np.ndarray, top: int) -> np.ndarray:
        _, mask = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)
        mask[:top, :] = 0
        # Only bridge one- or two-pixel camera/compression gaps.  This does
        # not create a large colour/edge candidate region.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    @staticmethod
    def _seed_from_histogram(histogram: np.ndarray, left: bool) -> int | None:
        midpoint = histogram.size // 2
        section = histogram[:midpoint] if left else histogram[midpoint:]
        if section.size == 0 or int(section.max()) == 0:
            return None
        return int(np.argmax(section)) + (0 if left else midpoint)

    def _track_direction(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int, direction: int,
    ) -> tuple[list[tuple[int, int]], list[tuple[int, int, int, bool]]]:
        if seed_x is None:
            return [], []
        height = max(8, (bottom - top) // self.window_count)
        x = float(seed_x)
        previous_x = x
        misses = 0
        points: list[tuple[int, int]] = []
        windows: list[tuple[int, int, int, bool]] = []
        y = seed_y
        while top <= y < bottom:
            y0, y1 = (
                (max(top, y - height), y) if direction < 0
                else (y, min(bottom, y + height))
            )
            if y1 <= y0:
                break
            window_x = int(round(x))
            count, _, stats, _ = cv2.connectedComponentsWithStats(
                mask[y0:y1, :], connectivity=8
            )
            candidates: list[tuple[float, float]] = []
            for label in range(1, count):
                area = int(stats[label, cv2.CC_STAT_AREA])
                component_width = int(stats[label, cv2.CC_STAT_WIDTH])
                if (
                    area < self.minimum_window_pixels
                    or area > self.maximum_component_pixels
                    or component_width > self.maximum_component_width_px
                ):
                    continue
                # Bounding-box midpoint, not an average of all white pixels.
                center_x = float(stats[label, cv2.CC_STAT_LEFT]) + 0.5 * component_width
                if abs(center_x - x) <= self.window_margin_px:
                    center_y = float(stats[label, cv2.CC_STAT_TOP] + y0) + 0.5 * float(stats[label, cv2.CC_STAT_HEIGHT])
                    candidates.append((center_x, center_y))
            if candidates:
                # Follow only the component closest to the predicted lane
                # position; background pixels in the same window cannot pull
                # the selected line by averaging.
                next_x, next_y = min(candidates, key=lambda item: abs(item[0] - x))
                points.append((int(round(next_x)), int(round(next_y))))
                previous_x, x = x, next_x
                misses = 0
                windows.append((window_x, y0, y1, True))
            else:
                windows.append((window_x, y0, y1, False))
                misses += 1
                if misses > self.maximum_missing_windows:
                    break
                x += np.clip(x - previous_x, -self.window_margin_px, self.window_margin_px)
            y += direction * height
        return points, windows

    def _track_lane(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int,
    ) -> tuple[np.ndarray, list[tuple[int, int, int, bool]]]:
        upward, up_windows = self._track_direction(
            mask, seed_x, seed_y, top, bottom, -1
        )
        downward, down_windows = self._track_direction(
            mask, seed_x, seed_y, top, bottom, 1
        )
        points = upward + downward
        return (
            np.asarray(points, dtype=np.int32)
            if points else np.empty((0, 2), dtype=np.int32),
            up_windows + down_windows,
        )

    @staticmethod
    def _fit(points: np.ndarray) -> np.ndarray | None:
        if points.shape[0] < 5:
            return None
        return np.polyfit(points[:, 1], points[:, 0], 2)

    @staticmethod
    def _image_message(source: Image, image: np.ndarray, encoding: str) -> Image:
        msg = Image()
        msg.header = source.header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = encoding
        msg.step = image.strides[0]
        msg.data = image.tobytes()
        return msg

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now < self.next_process_at:
            return
        self.next_process_at = now + 1.0 / self.processing_fps
        try:
            bgr = self._nv12_to_bgr(message)
            height, width = bgr.shape[:2]
            top = int(np.clip(self.roi_top_ratio * height, 0, height - 2))
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            mask = self._binary_white_mask(gray, top)
            seed_y = int(np.clip(self.seed_row_ratio * height, top, height - 1))
            band = self.seed_band_height_px // 2
            histogram = np.sum(mask[max(top, seed_y - band):min(height, seed_y + band + 1)] > 0, axis=0)
            left_seed = self._seed_from_histogram(histogram, True)
            right_seed = self._seed_from_histogram(histogram, False)
            left, left_windows = self._track_lane(mask, left_seed, seed_y, top, height)
            right, right_windows = self._track_lane(mask, right_seed, seed_y, top, height)

            overlay = bgr.copy()
            cv2.line(overlay, (0, top), (width - 1, top), (0, 165, 255), 2)
            cv2.line(overlay, (0, seed_y), (width - 1, seed_y), (0, 255, 255), 1)
            if left_seed is not None:
                cv2.circle(overlay, (left_seed, seed_y), 8, (255, 0, 0), -1)
            if right_seed is not None:
                cv2.circle(overlay, (right_seed, seed_y), 8, (0, 0, 255), -1)
            if self.show_diagnostic_windows:
                for windows, color in ((left_windows, (255, 150, 0)), (right_windows, (0, 150, 255))):
                    for x, y0, y1, found in windows:
                        cv2.rectangle(
                            overlay,
                            (max(0, x - self.window_margin_px), y0),
                            (min(width - 1, x + self.window_margin_px), y1),
                            color if found else (80, 80, 80), 1,
                        )
            for points, color in ((left, (255, 0, 0)), (right, (0, 0, 255))):
                if points.shape[0] >= 2:
                    ordered = points[np.argsort(points[:, 1])]
                    cv2.polylines(overlay, [ordered], False, color, 5, cv2.LINE_AA)
                    for point in ordered:
                        cv2.circle(overlay, tuple(point), 5, color, -1)

            left_fit, right_fit = self._fit(left), self._fit(right)
            confidence = 1.0 if left_fit is not None and right_fit is not None else 0.0
            lateral = lookahead = curvature = 0.0
            if confidence:
                center_fit = 0.5 * (left_fit + right_fit)
                reference_y = 0.75 * height
                lookahead_y = top + 0.25 * (height - top)
                lateral = (np.polyval(center_fit, reference_y) - width / 2) / (width / 2)
                lookahead = (np.polyval(center_fit, lookahead_y) - width / 2) / (width / 2)
                a, b, _ = center_fit
                slope = 2.0 * a * lookahead_y + b
                curvature = float(2.0 * a / ((1.0 + slope * slope) ** 1.5))

            if self.publish_debug_images:
                self.mask_pub.publish(self._image_message(message, mask, "mono8"))
                self.overlay_pub.publish(self._image_message(message, overlay, "bgr8"))
            if self.preview_enabled and now >= self.next_preview_at:
                cv2.imshow(self.preview_window_name, overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    self.stop_pub.publish(Bool(data=True))
                    self.get_logger().warn("SPACE pressed in preview: emergency stop requested.")
                self.next_preview_at = now + 1.0 / self.preview_fps
            self.model_pub.publish(Float32MultiArray(data=[
                float(confidence), float(lateral), float(lookahead), float(curvature),
                float(left_fit is not None), float(right_fit is not None),
            ]))
        except Exception as error:
            self.get_logger().error(f"Front lane frame rejected: {error}")

    def destroy_node(self) -> bool:
        if self.preview_enabled:
            cv2.destroyWindow(self.preview_window_name)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontLaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
