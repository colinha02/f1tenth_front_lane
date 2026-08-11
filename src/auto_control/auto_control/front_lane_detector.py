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
            "show_center_guidance": True,
            "close_target_y_ratio": 0.75,
            "far_target_y_ratio": 0.60,
            # Camera-visible reference: centre of the white tape on the front bumper.
            "vehicle_reference_x_ratio": 0.50,
            "vehicle_reference_y_ratio": 0.90,
            "width_profile_required_frames": 40,
            "width_profile_min_pairs_per_frame": 6,
            "width_profile_min_samples_per_bin": 12,
            "width_profile_max_samples_per_bin": 80,
            "width_profile_min_ratio": 0.10,
            "width_profile_max_ratio": 0.98,
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
        self.show_center_guidance = bool(value("show_center_guidance"))
        self.close_target_y_ratio = float(value("close_target_y_ratio"))
        self.far_target_y_ratio = float(value("far_target_y_ratio"))
        self.vehicle_reference_x_ratio = float(value("vehicle_reference_x_ratio"))
        self.vehicle_reference_y_ratio = float(value("vehicle_reference_y_ratio"))
        self.width_profile_required_frames = max(1, int(value("width_profile_required_frames")))
        self.width_profile_min_pairs_per_frame = max(2, int(value("width_profile_min_pairs_per_frame")))
        self.width_profile_min_samples_per_bin = max(1, int(value("width_profile_min_samples_per_bin")))
        self.width_profile_max_samples_per_bin = max(
            self.width_profile_min_samples_per_bin, int(value("width_profile_max_samples_per_bin"))
        )
        self.width_profile_min_ratio = float(value("width_profile_min_ratio"))
        self.width_profile_max_ratio = float(value("width_profile_max_ratio"))
        self.width_profile_y: np.ndarray | None = None
        self.width_profile_samples: list[list[float]] = []
        self.width_profile_frames = 0

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
    def _matched_lane_pairs(
        left: np.ndarray, right: np.ndarray, max_y_difference: int,
    ) -> np.ndarray:
        """Pair only neighbouring sliding-window points at similar heights.

        This deliberately uses measured midpoints rather than fitting a new
        curve, so the diagnostic path exposes the actual tracking result.
        """
        if left.size == 0 or right.size == 0:
            return np.empty((0, 3), dtype=np.int32)
        right_ordered = right[np.argsort(right[:, 1])]
        pairs: list[tuple[int, int, int]] = []
        for left_x, left_y in left[np.argsort(left[:, 1])]:
            nearest = int(np.argmin(np.abs(right_ordered[:, 1] - left_y)))
            right_x, right_y = right_ordered[nearest]
            if abs(int(right_y) - int(left_y)) > max_y_difference:
                continue
            lane_width = int(right_x) - int(left_x)
            if lane_width <= 0:
                continue
            pairs.append((
                int(round((int(left_x) + int(right_x)) / 2.0)),
                int(round((int(left_y) + int(right_y)) / 2.0)),
                lane_width,
            ))
        return np.asarray(pairs, dtype=np.int32) if pairs else np.empty((0, 3), dtype=np.int32)

    def _ensure_width_profile(self, top: int, bottom: int) -> None:
        if self.width_profile_y is not None:
            return
        window_height = max(8, (bottom - top) // self.window_count)
        self.width_profile_y = np.arange(
            top + window_height // 2, bottom, window_height, dtype=np.int32
        )
        self.width_profile_samples = [[] for _ in self.width_profile_y]

    def _profile_width_at(self, y: int) -> float | None:
        if self.width_profile_y is None or not self.width_profile_samples:
            return None
        index = int(np.argmin(np.abs(self.width_profile_y - y)))
        samples = self.width_profile_samples[index]
        if len(samples) < self.width_profile_min_samples_per_bin:
            return None
        return float(np.median(samples))

    def _profile_ready(self) -> bool:
        return (
            self.width_profile_y is not None
            and self.width_profile_frames >= self.width_profile_required_frames
            and all(len(samples) >= self.width_profile_min_samples_per_bin
                    for samples in self.width_profile_samples)
        )

    def _learn_width_profile(self, pairs: np.ndarray, width: int) -> None:
        """Accept only coherent two-lane frames; use a median per image height."""
        if self.width_profile_y is None or pairs.shape[0] < self.width_profile_min_pairs_per_frame:
            return
        valid = [pair for pair in pairs if (
            self.width_profile_min_ratio * width <= pair[2] <= self.width_profile_max_ratio * width
        )]
        if len(valid) < self.width_profile_min_pairs_per_frame:
            return
        self.width_profile_frames += 1
        for _, y, lane_width in valid:
            index = int(np.argmin(np.abs(self.width_profile_y - y)))
            samples = self.width_profile_samples[index]
            samples.append(float(lane_width))
            if len(samples) > self.width_profile_max_samples_per_bin:
                del samples[0]

    def _virtual_centers(
        self, left: np.ndarray, right: np.ndarray, real_centers: np.ndarray,
        max_y_difference: int,
    ) -> np.ndarray:
        """Use a learned, perspective-aware width only where a side is missing."""
        if not self._profile_ready():
            return np.empty((0, 2), dtype=np.int32)
        virtual: list[tuple[int, int]] = []

        def has_real_center(y: int) -> bool:
            return bool(real_centers.size and np.min(np.abs(real_centers[:, 1] - y)) <= max_y_difference)

        for points, side in ((left, "left"), (right, "right")):
            for x, y in points:
                if has_real_center(int(y)):
                    continue
                lane_width = self._profile_width_at(int(y))
                if lane_width is None:
                    continue
                center_x = int(round(float(x) + lane_width / 2.0)) if side == "left" else int(round(float(x) - lane_width / 2.0))
                virtual.append((center_x, int(y)))
        return np.asarray(virtual, dtype=np.int32) if virtual else np.empty((0, 2), dtype=np.int32)

    @staticmethod
    def _nearest_y_point(points: np.ndarray, target_y: int) -> tuple[int, int] | None:
        if points.size == 0:
            return None
        index = int(np.argmin(np.abs(points[:, 1] - target_y)))
        return tuple(map(int, points[index]))

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
            window_height = max(8, (height - top) // self.window_count)
            self._ensure_width_profile(top, height)
            pairs = self._matched_lane_pairs(left, right, max_y_difference=window_height)
            self._learn_width_profile(pairs, width)
            real_centers = pairs[:, :2] if pairs.size else np.empty((0, 2), dtype=np.int32)
            virtual_centers = self._virtual_centers(
                left, right, real_centers, max_y_difference=window_height
            )
            centers = np.vstack((real_centers, virtual_centers)) if virtual_centers.size else real_centers
            close_target_y = int(np.clip(self.close_target_y_ratio * height, top, height - 1))
            far_target_y = int(np.clip(self.far_target_y_ratio * height, top, height - 1))
            close_target = self._nearest_y_point(centers, close_target_y)
            far_target = self._nearest_y_point(centers, far_target_y)

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

            if self.show_center_guidance:
                # Do not move the lane centreline to compensate for the camera
                # viewpoint.  Use the measured bumper-tape position as the
                # vehicle reference from which the guidance line starts.
                vehicle_point = (
                    int(np.clip(self.vehicle_reference_x_ratio * width, 0, width - 1)),
                    int(np.clip(self.vehicle_reference_y_ratio * height, 0, height - 1)),
                )
                if centers.shape[0] >= 2:
                    ordered_centers = centers[np.argsort(centers[:, 1])]
                    cv2.polylines(overlay, [ordered_centers], False, (255, 255, 0), 3, cv2.LINE_AA)
                    for point in real_centers[np.argsort(real_centers[:, 1])]:
                        cv2.circle(overlay, tuple(point), 4, (255, 255, 0), -1)
                    for point in virtual_centers:
                        cv2.circle(overlay, tuple(point), 5, (255, 0, 255), -1)
                cv2.circle(overlay, vehicle_point, 10, (0, 255, 0), -1)
                cv2.putText(overlay, "vehicle", (vehicle_point[0] + 12, vehicle_point[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
                if close_target is not None:
                    cv2.circle(overlay, close_target, 9, (0, 255, 255), -1)
                    cv2.line(overlay, vehicle_point, close_target, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(overlay, "close", (close_target[0] + 10, close_target[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
                if far_target is not None:
                    cv2.circle(overlay, far_target, 9, (255, 0, 255), -1)
                    if close_target is not None:
                        cv2.line(overlay, close_target, far_target, (0, 255, 0), 2, cv2.LINE_AA)
                    else:
                        cv2.line(overlay, vehicle_point, far_target, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(overlay, "far", (far_target[0] + 10, far_target[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)
                profile_state = "READY" if self._profile_ready() else f"learning {self.width_profile_frames}/{self.width_profile_required_frames}"
                cv2.putText(overlay, f"center pairs: {real_centers.shape[0]}  virtual: {virtual_centers.shape[0]}", (20, height - 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"width profile: {profile_state}", (20, height - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

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
