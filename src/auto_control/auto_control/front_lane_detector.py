#!/usr/bin/env python3
"""Direct front-camera lane detector.

The detector deliberately works in image coordinates.  It uses the camera
driver's rectified NV12 stream but does not calculate or consume a BEV image.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray


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
        self._emergency_stop_pub = self.create_publisher(
            Bool, "/auto/emergency_stop", qos
        )
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
            "preview_enabled": True,
            "preview_window_name": "Front lane detection",
            "preview_fps": 30.0,
            "processing_fps": 30.0,
            "publish_debug_images": False,
            "show_extracted_lane_mask": True,
            "extracted_lane_mask_alpha": 0.55,
            "show_model_paths": False,
            "minimum_displayed_lane_vertical_coverage": 0.20,
            "displayed_lane_trace_thickness_px": 7,
            "roi_top_ratio": 0.5,
            "roi_bottom_ratio": 1.0,
            "window_count": 12,
            "window_margin_px": 85,
            "seed_histogram_smoothing_px": 31,
            "window_prediction_max_step_px": 55,
            "candidate_path_count": 16,
            "minimum_component_pixels": 8,
            "maximum_component_pixels": 1200,
            "maximum_component_width_px": 100,
            "minimum_window_pixels": 35,
            "minimum_fit_pixels": 180,
            "sobel_edge_threshold": 55,
            "centerline_bin_height_px": 12,
            "centerline_outlier_px": 24.0,
            "white_lightness_min": 82,
            "white_saturation_max": 179,
            "track_dark_lightness_max": 42,
            "dark_adjacency_kernel_px": 75,
            "morphology_kernel_px": 7,
            "noise_opening_kernel_px": 3,
            "minimum_fit_vertical_coverage_ratio": 0.45,
            "temporal_alpha": 0.70,
            "temporal_maximum_jump_px": 95.0,
            "temporal_hold_frames": 3,
            "expected_lane_width_px": 360.0,
            "minimum_lane_width_px": 180.0,
            "maximum_lane_width_px": 650.0,
            "lane_width_temporal_alpha": 0.25,
            "control_reference_y_ratio": 0.67,
            "lookahead_ratio": 0.48,
        }.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        parameter = lambda name: self.get_parameter(name).value
        self.image_topic = str(parameter("image_topic"))
        self.mask_topic = str(parameter("mask_topic"))
        self.overlay_topic = str(parameter("overlay_topic"))
        self.model_topic = str(parameter("model_topic"))
        self.preview_enabled = bool(parameter("preview_enabled"))
        self.preview_window_name = str(parameter("preview_window_name"))
        self.preview_fps = max(1.0, float(parameter("preview_fps")))
        self.processing_fps = max(1.0, float(parameter("processing_fps")))
        self.publish_debug_images = bool(parameter("publish_debug_images"))
        self.show_extracted_lane_mask = bool(parameter("show_extracted_lane_mask"))
        self.extracted_lane_mask_alpha = float(parameter("extracted_lane_mask_alpha"))
        self.show_model_paths = bool(parameter("show_model_paths"))
        self.minimum_displayed_lane_vertical_coverage = float(
            parameter("minimum_displayed_lane_vertical_coverage")
        )
        self.displayed_lane_trace_thickness_px = max(
            1, int(parameter("displayed_lane_trace_thickness_px"))
        )
        self.roi_top_ratio = float(parameter("roi_top_ratio"))
        self.roi_bottom_ratio = float(parameter("roi_bottom_ratio"))
        self.window_count = max(4, int(parameter("window_count")))
        self.window_margin_px = max(10, int(parameter("window_margin_px")))
        self.seed_histogram_smoothing_px = max(
            3, int(parameter("seed_histogram_smoothing_px"))
        )
        if self.seed_histogram_smoothing_px % 2 == 0:
            self.seed_histogram_smoothing_px += 1
        self.window_prediction_max_step_px = max(
            1, int(parameter("window_prediction_max_step_px"))
        )
        self.candidate_path_count = max(2, int(parameter("candidate_path_count")))
        self.minimum_component_pixels = max(
            1, int(parameter("minimum_component_pixels"))
        )
        self.maximum_component_pixels = max(
            self.minimum_component_pixels,
            int(parameter("maximum_component_pixels")),
        )
        self.maximum_component_width_px = max(
            5, int(parameter("maximum_component_width_px"))
        )
        self.minimum_window_pixels = max(5, int(parameter("minimum_window_pixels")))
        self.minimum_fit_pixels = max(30, int(parameter("minimum_fit_pixels")))
        self.sobel_edge_threshold = max(1, int(parameter("sobel_edge_threshold")))
        self.centerline_bin_height_px = max(
            4, int(parameter("centerline_bin_height_px"))
        )
        self.centerline_outlier_px = max(
            2.0, float(parameter("centerline_outlier_px"))
        )
        self.white_lightness_min = int(parameter("white_lightness_min"))
        self.white_saturation_max = int(parameter("white_saturation_max"))
        self.track_dark_lightness_max = int(parameter("track_dark_lightness_max"))
        self.dark_adjacency_kernel_px = max(
            1, int(parameter("dark_adjacency_kernel_px"))
        )
        if self.dark_adjacency_kernel_px % 2 == 0:
            self.dark_adjacency_kernel_px += 1
        self.morphology_kernel_px = max(1, int(parameter("morphology_kernel_px")))
        if self.morphology_kernel_px % 2 == 0:
            self.morphology_kernel_px += 1
        self.noise_opening_kernel_px = max(
            1, int(parameter("noise_opening_kernel_px"))
        )
        if self.noise_opening_kernel_px % 2 == 0:
            self.noise_opening_kernel_px += 1
        self.minimum_fit_vertical_coverage_ratio = float(
            parameter("minimum_fit_vertical_coverage_ratio")
        )
        self.temporal_alpha = float(parameter("temporal_alpha"))
        self.temporal_maximum_jump_px = float(parameter("temporal_maximum_jump_px"))
        self.temporal_hold_frames = max(0, int(parameter("temporal_hold_frames")))
        self.expected_lane_width_px = float(parameter("expected_lane_width_px"))
        self.minimum_lane_width_px = float(parameter("minimum_lane_width_px"))
        self.maximum_lane_width_px = float(parameter("maximum_lane_width_px"))
        self.lane_width_temporal_alpha = float(parameter("lane_width_temporal_alpha"))
        self.control_reference_y_ratio = float(parameter("control_reference_y_ratio"))
        self.lookahead_ratio = float(parameter("lookahead_ratio"))
        self._next_preview_at = 0.0
        self._next_process_at = 0.0

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
        white_mask = cv2.inRange(hls, lower, upper)
        dark_mask = cv2.inRange(
            hls,
            np.array([0, 0, 0], dtype=np.uint8),
            np.array([179, self.track_dark_lightness_max, 255], dtype=np.uint8),
        )
        # The old 75px elliptical dilation consumed most of a frame on the
        # Jetson.  A distance transform expresses the same rule (white tape
        # must be within the kernel radius of dark track) much faster.
        distance_to_dark = cv2.distanceTransform(
            cv2.bitwise_not(dark_mask), cv2.DIST_L2, 3
        )
        dark_neighborhood = np.where(
            distance_to_dark <= 0.5 * self.dark_adjacency_kernel_px,
            255,
            0,
        ).astype(np.uint8)
        # Preserve the full white tape core.  Background white areas without
        # nearby black track are removed before path scoring.
        mask = cv2.bitwise_and(white_mask, dark_neighborhood)
        roi_mask = np.zeros_like(mask)
        roi_mask[top:bottom, :] = 255
        mask = cv2.bitwise_and(mask, roi_mask)
        opening_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.noise_opening_kernel_px, self.noise_opening_kernel_px),
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.morphology_kernel_px, self.morphology_kernel_px),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    @staticmethod
    def _skeletonize(mask: np.ndarray) -> np.ndarray:
        """Reduce each extracted tape strip to its actual one-pixel center."""
        work = mask.copy()
        skeleton = np.zeros_like(mask)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while cv2.countNonZero(work) > 0:
            opened = cv2.morphologyEx(work, cv2.MORPH_OPEN, element)
            skeleton = cv2.bitwise_or(skeleton, cv2.subtract(work, opened))
            work = cv2.erode(work, element)
        return skeleton

    def _displayable_lane_skeleton(
        self, skeleton: np.ndarray, top: int, bottom: int
    ) -> np.ndarray:
        labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            skeleton, connectivity=8
        )
        kept = np.zeros_like(skeleton)
        minimum_span = self.minimum_displayed_lane_vertical_coverage * (bottom - top)
        for label in range(1, labels_count):
            y = stats[label, cv2.CC_STAT_TOP]
            height = stats[label, cv2.CC_STAT_HEIGHT]
            bottom_y = y + height - 1
            # A lane must be long in the ROI and reach its lower part.  Short
            # horizontal wall/tile fragments therefore remain candidates for
            # tracking but are not presented as detected lane lines.
            if height >= minimum_span and bottom_y >= top + 0.45 * (bottom - top):
                kept[labels == label] = 255
        return kept

    def _sliding_window_points(
        self,
        mask: np.ndarray,
        start_x: Optional[int],
        reference_y: int,
        top: int,
        bottom: int,
        left_side: bool,
        previous_fit: Optional[np.ndarray],
    ) -> np.ndarray:
        height = max(1, (bottom - top) // self.window_count)
        windows: list[list[tuple[np.ndarray, float, float]]] = []
        for index in range(self.window_count):
            y_high = bottom - index * height
            y_low = max(top, y_high - height)
            window = mask[y_low:y_high, :]
            labels_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
                window, connectivity=8
            )
            components: list[tuple[np.ndarray, float, float]] = []
            for label in range(1, labels_count):
                area = int(stats[label, cv2.CC_STAT_AREA])
                # Edge-based tape masks are often split into thin pieces in a
                # single window.  Keep small pieces as candidates; the full
                # path score, not this local area gate, rejects background.
                component_width = int(stats[label, cv2.CC_STAT_WIDTH])
                if (
                    area < self.minimum_component_pixels
                    or area > self.maximum_component_pixels
                    or component_width > self.maximum_component_width_px
                ):
                    continue
                component_y, component_x = np.nonzero(labels == label)
                points = np.column_stack((component_y + y_low, component_x))
                components.append((points, float(centroids[label, 0]), float(area)))
            windows.append(components)

        if start_x is None or not windows:
            return np.empty((0, 2), dtype=np.float64)

        # The histogram seed is deliberately taken at the 75%-height row.
        # Starting at the image bottom makes tight curves latch onto a small
        # background fragment before reaching the actual tape.
        anchor_index = next(
            (
                index for index in range(self.window_count)
                if bottom - (index + 1) * height <= reference_y < bottom - index * height
            ),
            self.window_count - 1,
        )
        midpoint = 0.5 * mask.shape[1]
        start_candidates = [
            component for component in windows[anchor_index]
            if (component[1] < midpoint) == left_side
        ]
        seed_radius = max(2.0 * self.window_margin_px, 130.0)
        nearby = [
            component for component in start_candidates
            if abs(component[1] - start_x) <= seed_radius
        ]
        if nearby:
            start_candidates = nearby
        if not start_candidates:
            return np.empty((0, 2), dtype=np.float64)

        # Each hypothesis stores score, selected components, their x centers,
        # and its number of tolerated missing bands.
        hypotheses: list[tuple[float, list[np.ndarray], list[float], int]] = []
        for points, center_x, area in start_candidates:
            anchor_cost = 0.08 * abs(center_x - start_x)
            prior_cost = 0.0
            if previous_fit is not None:
                prior_cost = 0.04 * abs(
                    center_x - np.polyval(previous_fit, np.mean(points[:, 0]))
                )
            hypotheses.append((
                np.log1p(area) - anchor_cost - prior_cost,
                [points], [center_x], 0,
            ))
        hypotheses.sort(key=lambda item: item[0], reverse=True)
        hypotheses = hypotheses[:self.candidate_path_count]

        def extend(direction: int) -> list[np.ndarray]:
            paths = hypotheses
            for index in range(anchor_index + direction, len(windows) if direction > 0 else -1, direction):
                components = windows[index]
                expanded: list[tuple[float, list[np.ndarray], list[float], int]] = []
                for score, path_points, centers, misses in paths:
                    if not components:
                        if misses < 2:
                            expanded.append((score - 2.0, path_points, centers, misses + 1))
                        continue
                    previous_step = centers[-1] - centers[-2] if len(centers) > 1 else 0.0
                    predicted_x = centers[-1] + np.clip(
                        previous_step,
                        -self.window_prediction_max_step_px,
                        self.window_prediction_max_step_px,
                    )
                    # Permit large sideways movement on a tight curve, but
                    # only when the preceding path already shows that motion.
                    maximum_transition = max(
                        2.0 * self.window_margin_px,
                        self.window_margin_px + 1.8 * abs(previous_step),
                    )
                    for points, center_x, area in components:
                        transition = abs(center_x - predicted_x)
                        if transition > maximum_transition:
                            continue
                        prior_cost = 0.0
                        if previous_fit is not None:
                            prior_cost = 0.025 * abs(
                                center_x - np.polyval(previous_fit, np.mean(points[:, 0]))
                            )
                        expanded.append((
                            score + np.log1p(area) - 0.055 * transition - prior_cost,
                            path_points + [points], centers + [center_x], 0,
                        ))
                if not expanded:
                    break
                expanded.sort(key=lambda item: item[0], reverse=True)
                paths = expanded[:self.candidate_path_count]
            return paths[0][1] if paths else []

        # Trace upward and downward from the *same* trusted seed, then merge
        # both observed tape sections.  This supports a lane disappearing at
        # the lower image edge during a very tight curve.
        points = extend(1) + extend(-1)
        return np.vstack(points).astype(np.float64) if points else np.empty((0, 2), dtype=np.float64)

    def _seeded_sliding_points(self, mask, seed_x, reference_y, top, bottom):
        if seed_x is None:
            return np.empty((0, 2), dtype=np.float64)
        nonzero_y, nonzero_x = mask.nonzero()
        result = []
        for direction in (-1, 1):
            current_x, y = int(seed_x), reference_y
            previous_x, misses = float(current_x), 0
            while top <= y < bottom:
                y0, y1 = (max(top, y - 24), y) if direction < 0 else (y, min(bottom, y + 24))
                margin = max(self.window_margin_px, 170)
                inside = ((nonzero_y >= y0) & (nonzero_y < y1) & (nonzero_x >= current_x - margin) & (nonzero_x <= current_x + margin))
                ids = np.flatnonzero(inside)
                if ids.size < self.minimum_window_pixels:
                    misses += 1
                    if misses > 3:
                        break
                    current_x = int(round(current_x + np.clip(current_x - previous_x, -60, 60)))
                    y += direction * 24
                    continue
                result.append(np.column_stack((nonzero_y[ids], nonzero_x[ids])))
                previous_x, current_x = float(current_x), int(np.mean(nonzero_x[ids]))
                misses = 0
                y += direction * 24
        return np.vstack(result).astype(np.float64) if result else np.empty((0, 2), dtype=np.float64)

    @staticmethod
    def _connected_lane_points(skeleton, seed_x, reference_y):
        if seed_x is None:
            return np.empty((0, 2), dtype=np.float64)
        count, labels, _, _ = cv2.connectedComponentsWithStats(skeleton, connectivity=8)
        best = None
        for label in range(1, count):
            ys, xs = np.nonzero(labels == label)
            if ys.size == 0:
                continue
            distance = np.min((xs - seed_x) ** 2 + (ys - reference_y) ** 2)
            if best is None or distance < best[0]:
                best = (distance, ys, xs)
        if best is None or best[0] > 70 * 70:
            return np.empty((0, 2), dtype=np.float64)
        return np.column_stack((best[1], best[2])).astype(np.float64)

    @staticmethod
    def _row_centerline(left, right, top, bottom):
        centers, previous_center, previous_left, previous_right = [], None, None, None
        previous_step, previous_half_width, missing_rows = 0.0, None, 0
        for y in range(bottom - 1, top - 1, -4):
            def row_x(points):
                if points.size == 0: return None
                values = points[np.abs(points[:, 0] - y) <= 2, 1]
                return float(np.median(values)) if values.size else None
            lx, rx = row_x(left), row_x(right)
            if lx is not None and rx is not None:
                center = 0.5 * (lx + rx)
                half_width = 0.5 * (rx - lx)
                if half_width > 15.0:
                    previous_half_width = half_width if previous_half_width is None else (
                        0.75 * previous_half_width + 0.25 * half_width
                    )
            elif lx is not None and previous_center is not None and previous_left is not None:
                center = lx + previous_half_width if previous_half_width is not None else (
                    previous_center + (lx - previous_left)
                )
            elif rx is not None and previous_center is not None and previous_right is not None:
                center = rx - previous_half_width if previous_half_width is not None else (
                    previous_center + (rx - previous_right)
                )
            elif previous_center is not None and missing_rows < 2:
                center = previous_center + np.clip(previous_step, -28.0, 28.0)
            else:
                center = None
            if center is not None: centers.append((int(round(center)), y))
            if lx is not None: previous_left = lx
            if rx is not None: previous_right = rx
            if center is not None and previous_center is not None:
                previous_step = center - previous_center
            previous_center = center
            missing_rows = 0 if (lx is not None or rx is not None) else missing_rows + 1
        return np.asarray(centers, dtype=np.int32)

    @staticmethod
    def _points_mask(points: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
        selected = np.zeros(shape, dtype=np.uint8)
        if points.size:
            ys = np.clip(points[:, 0].astype(np.int32), 0, shape[0] - 1)
            xs = np.clip(points[:, 1].astype(np.int32), 0, shape[1] - 1)
            selected[ys, xs] = 255
        return selected

    def _tape_edge_centers(
        self, gray: np.ndarray, selected: np.ndarray, top: int, bottom: int
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return Sobel edge pixels and the midpoint between each edge pair."""
        if cv2.countNonZero(selected) == 0:
            return np.empty((0, 2), dtype=np.float64), np.empty((0, 2), dtype=np.int32)
        gradient = cv2.convertScaleAbs(
            cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3)
        )
        # Search only a few pixels around the already selected continuous tape
        # path.  This prevents wall/tile edges from becoming lane boundaries.
        search = cv2.dilate(
            selected, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        )
        edge_mask = np.where(
            (gradient >= self.sobel_edge_threshold) & (search > 0), 255, 0
        ).astype(np.uint8)
        centers: list[tuple[float, float]] = []
        for y in range(bottom - 1, top - 1, -2):
            tape_x = np.flatnonzero(selected[y])
            if tape_x.size < 2:
                continue
            left_limit, right_limit = int(tape_x.min()), int(tape_x.max())
            edge_x = np.flatnonzero(edge_mask[y])
            if edge_x.size:
                left_candidates = edge_x[np.abs(edge_x - left_limit) <= 5]
                right_candidates = edge_x[np.abs(edge_x - right_limit) <= 5]
                left_edge = int(left_candidates[np.argmin(np.abs(left_candidates - left_limit))]) if left_candidates.size else left_limit
                right_edge = int(right_candidates[np.argmin(np.abs(right_candidates - right_limit))]) if right_candidates.size else right_limit
            else:
                left_edge, right_edge = left_limit, right_limit
            if right_edge - left_edge >= 2:
                centers.append((float(y), 0.5 * (left_edge + right_edge)))
        return np.column_stack(np.nonzero(edge_mask)).astype(np.float64), np.asarray(
            [(int(round(x)), int(round(y))) for y, x in centers], dtype=np.int32
        )

    def _smooth_centerline(
        self, centers: np.ndarray, top: int, bottom: int
    ) -> np.ndarray:
        """Bin row centers, reject outliers, then render one smooth curve."""
        if centers.shape[0] < 4:
            return centers
        bins: list[tuple[float, float]] = []
        for y0 in range(top, bottom, self.centerline_bin_height_px):
            values = centers[
                (centers[:, 1] >= y0) &
                (centers[:, 1] < y0 + self.centerline_bin_height_px)
            ]
            if values.size:
                bins.append((
                    float(np.median(values[:, 0])),
                    float(np.median(values[:, 1])),
                ))
        samples = np.asarray(bins, dtype=np.float64)
        if samples.shape[0] < 4:
            return centers
        degree = 2 if samples.shape[0] >= 6 else 1
        fit = np.polyfit(samples[:, 1], samples[:, 0], degree)
        residual = np.abs(samples[:, 0] - np.polyval(fit, samples[:, 1]))
        inliers = residual <= self.centerline_outlier_px
        if int(np.count_nonzero(inliers)) >= degree + 2:
            fit = np.polyfit(samples[inliers, 1], samples[inliers, 0], degree)
            samples = samples[inliers]
        y_start = int(np.clip(np.max(samples[:, 1]), top, bottom - 1))
        y_end = int(np.clip(np.min(samples[:, 1]), top, bottom - 1))
        ys = np.arange(y_start, y_end - 1, -4, dtype=np.float64)
        return np.column_stack((np.polyval(fit, ys), ys)).astype(np.int32)

    @staticmethod
    def _lane_x_near_row(
        points: np.ndarray, row: int, maximum_distance_px: int = 180
    ) -> Optional[float]:
        if points.size == 0:
            return None
        distances = np.abs(points[:, 0] - row)
        nearest = float(np.min(distances))
        if nearest > maximum_distance_px:
            return None
        # Use the nearest observed row band; the sliding path may end a few
        # pixels below the orange ROI line on a tight bend.
        values = points[distances <= nearest + 4.0, 1]
        return float(np.median(values)) if values.size else None

    @staticmethod
    def _fit(
        points: np.ndarray,
        minimum_pixels: int,
        minimum_vertical_span_px: float,
    ) -> Optional[np.ndarray]:
        if points.shape[0] < minimum_pixels:
            return None
        if float(np.ptp(points[:, 0])) < minimum_vertical_span_px:
            return None
        return np.polyfit(points[:, 0], points[:, 1], 2)

    def _initial_seed(
        self,
        histogram: np.ndarray,
        left: bool,
        previous: Optional[np.ndarray],
        bottom: int,
    ) -> Optional[int]:
        smoothed = cv2.GaussianBlur(
            histogram.astype(np.float32).reshape(1, -1),
            (self.seed_histogram_smoothing_px, 1), 0,
        ).reshape(-1)
        midpoint = smoothed.size // 2
        section = smoothed[:midpoint] if left else smoothed[midpoint:]
        offset = 0 if left else midpoint
        if previous is not None:
            # Once a lane was found, continue from its predicted position
            # instead of allowing a bright background object to replace it.
            expected_x = int(np.clip(np.polyval(previous, bottom - 1), 0, smoothed.size - 1))
            search_radius = self.window_margin_px
            lo = max(offset, expected_x - search_radius)
            hi = min(offset + section.size, expected_x + search_radius + 1)
            nearby = smoothed[lo:hi]
            if nearby.size and int(nearby.max()) > 0:
                return int(np.argmax(nearby)) + lo
            return expected_x
        if section.size and int(section.max()) > 0:
            return int(np.argmax(section)) + offset
        return None

    def _accept_fit(
        self, candidate: Optional[np.ndarray], state: FitState, sample_y: np.ndarray
    ) -> tuple[Optional[np.ndarray], bool]:
        if candidate is not None:
            if state.coefficients is None:
                state.coefficients = candidate
                state.held_frames = 0
                return candidate, True
            differences = np.abs(
                np.polyval(candidate, sample_y) - np.polyval(state.coefficients, sample_y)
            )
            # A background line can agree near the vehicle but bend away at
            # the far end.  Use an upper-percentile difference rather than a
            # median so that far-field hijacking is rejected.
            jump = float(np.percentile(differences, 80.0))
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
        now = time.monotonic()
        if now < self._next_process_at:
            return
        self._next_process_at = now + 1.0 / self.processing_fps
        try:
            bgr = self._nv12_to_bgr(message)
            height, width = bgr.shape[:2]
            top = int(np.clip(self.roi_top_ratio * height, 0, height - 2))
            bottom = int(np.clip(self.roi_bottom_ratio * height, top + 2, height))
            mask = self._candidate_mask(bgr, top, bottom)
            reference_row = int(np.clip(0.75 * height, top, bottom - 1))
            histogram = np.sum(mask[max(top, reference_row - 8):min(bottom, reference_row + 9)] > 0, axis=0)
            sample_y = np.linspace(bottom - 1, top, 8)
            left_seed = self._initial_seed(
                histogram, True, self._left_state.coefficients, reference_row
            )
            right_seed = self._initial_seed(
                histogram, False, self._right_state.coefficients, reference_row
            )
            # Do not select a whole connected component merely because it is
            # close to a seed: a wall/tile marking can be connected to it at a
            # curve.  The sliding-window beam keeps only the continuous
            # bottom-to-top path that agrees with the seed and previous path.
            # It also avoids the iterative full-frame skeletonization that was
            # the main source of preview lag on the Jetson.
            left_points = self._sliding_window_points(
                mask, left_seed, reference_row, top, bottom, True,
                self._left_state.coefficients
            )
            right_points = self._sliding_window_points(
                mask, right_seed, reference_row, top, bottom, False,
                self._right_state.coefficients
            )
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            left_selected = self._points_mask(left_points, mask.shape)
            right_selected = self._points_mask(right_points, mask.shape)
            left_edge_points, left_tape_centers = self._tape_edge_centers(
                gray, left_selected, top, bottom
            )
            right_edge_points, right_tape_centers = self._tape_edge_centers(
                gray, right_selected, top, bottom
            )
            # Centerline is calculated from each tape's two Sobel boundaries,
            # rather than from every filled white pixel in the tape strip.
            left_center_points = (
                np.column_stack((left_tape_centers[:, 1], left_tape_centers[:, 0]))
                if left_tape_centers.size else np.empty((0, 2), dtype=np.float64)
            )
            right_center_points = (
                np.column_stack((right_tape_centers[:, 1], right_tape_centers[:, 0]))
                if right_tape_centers.size else np.empty((0, 2), dtype=np.float64)
            )
            left_roi_x = self._lane_x_near_row(left_center_points, top)
            right_roi_x = self._lane_x_near_row(right_center_points, top)
            roi_center_x = (
                0.5 * (left_roi_x + right_roi_x)
                if left_roi_x is not None and right_roi_x is not None
                else None
            )
            minimum_vertical_span_px = (
                self.minimum_fit_vertical_coverage_ratio * (bottom - top)
            )
            left_fit, left_detected = self._accept_fit(
                self._fit(
                    left_center_points, self.minimum_fit_pixels, minimum_vertical_span_px
                ), self._left_state, sample_y
            )
            right_fit, right_detected = self._accept_fit(
                self._fit(
                    right_center_points, self.minimum_fit_pixels, minimum_vertical_span_px
                ), self._right_state, sample_y
            )

            if left_detected and right_detected and left_fit is not None and right_fit is not None:
                width_fit = right_fit - left_fit
                lane_widths = np.polyval(width_fit, sample_y)
                if np.all((lane_widths >= self.minimum_lane_width_px) & (lane_widths <= self.maximum_lane_width_px)):
                    if self._width_fit is None:
                        self._width_fit = width_fit
                    else:
                        self._width_fit = (
                            self.lane_width_temporal_alpha * width_fit
                            + (1.0 - self.lane_width_temporal_alpha) * self._width_fit
                        )

            width_fit = self._width_fit
            if width_fit is None:
                width_fit = np.array([0.0, 0.0, self.expected_lane_width_px])
            # Use the currently observed boundary immediately.  A held fit on
            # the missing side is less useful than a virtual boundary made
            # from the visible tape and the learned width profile.
            left_inferred = False
            right_inferred = False
            if right_detected and not left_detected:
                left_fit = right_fit - width_fit
                left_inferred = left_fit is not None
            elif left_detected and not right_detected:
                right_fit = left_fit + width_fit
                right_inferred = right_fit is not None
            elif left_fit is None and right_fit is not None:
                left_fit = right_fit - width_fit
                left_inferred = left_fit is not None
            elif right_fit is None and left_fit is not None:
                right_fit = left_fit + width_fit
                right_inferred = right_fit is not None

            overlay = bgr.copy()
            tracked_pixels = cv2.bitwise_or(left_selected, right_selected)
            # Solid red means this exact white-mask pixel was accepted by a
            # lane tracking path.  Unselected background candidates are not
            # painted on the source image.
            tracked_trace = cv2.dilate(
                tracked_pixels,
                # This affects only the visualization.  It joins tiny holes
                # between consecutive selected windows without treating a new
                # background object as part of the lane path.
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
            )
            overlay[tracked_trace > 0] = (0, 0, 255)
            for edge_points in (left_edge_points, right_edge_points):
                if edge_points.size:
                    pixel_y = edge_points[:, 0].astype(np.int32)
                    pixel_x = edge_points[:, 1].astype(np.int32)
                    overlay[pixel_y, pixel_x] = (255, 255, 0)
            if roi_center_x is not None:
                bottom_anchor = (width // 2, bottom - 14)
                roi_anchor = (int(round(roi_center_x)), top)
                cv2.line(overlay, bottom_anchor, roi_anchor, (0, 255, 0), 4)
                cv2.circle(overlay, bottom_anchor, 8, (0, 255, 0), -1)
                cv2.circle(overlay, roi_anchor, 8, (0, 255, 0), -1)
            cv2.line(overlay, (0, top), (width - 1, top), (0, 165, 255), 2)
            ys = np.linspace(bottom - 1, top, 80)
            left_curve = self._points_for_fit(left_fit, ys, width)
            right_curve = self._points_for_fit(right_fit, ys, width)
            if self.show_model_paths and left_curve is not None:
                cv2.polylines(
                    overlay, [left_curve], False,
                    (255, 255, 0) if left_inferred else (255, 0, 0), 5,
                )
            if self.show_model_paths and right_curve is not None:
                cv2.polylines(
                    overlay, [right_curve], False,
                    (255, 255, 0) if right_inferred else (255, 0, 0), 5,
                )

            valid_sides = int(left_fit is not None) + int(right_fit is not None)
            confidence = 0.0
            lateral_error = 0.0
            lookahead_offset = 0.0
            curvature = 0.0
            if left_fit is not None and right_fit is not None:
                center_fit = 0.5 * (left_fit + right_fit)
                center_curve = self._points_for_fit(center_fit, ys, width)
                if self.show_model_paths and center_curve is not None:
                    cv2.polylines(overlay, [center_curve], False, (0, 255, 0), 3)
                reference_y = float(np.clip(
                    self.control_reference_y_ratio * height, top, bottom - 1
                ))
                lookahead_y = top + self.lookahead_ratio * (bottom - top)
                reference_x = float(np.polyval(center_fit, reference_y))
                lookahead_x = float(np.polyval(center_fit, lookahead_y))
                lateral_error = (reference_x - 0.5 * width) / (0.5 * width)
                lookahead_offset = (lookahead_x - 0.5 * width) / (0.5 * width)
                if self.show_model_paths:
                    cv2.circle(
                        overlay, (int(round(reference_x)), int(round(reference_y))),
                        7, (255, 0, 255), -1,
                    )
                    cv2.circle(
                        overlay, (width // 2, int(round(reference_y))),
                        6, (0, 255, 255), 2,
                    )
                a, b, _ = center_fit
                slope = 2.0 * a * lookahead_y + b
                curvature = float(2.0 * a / ((1.0 + slope * slope) ** 1.5))
                actual_sides = int(left_detected) + int(right_detected)
                confidence = 1.0 if actual_sides == 2 else 0.60

            # A 1280x720 BGR ROS image is several MB per frame.  The OpenCV
            # preview below is enough during driving, so do not serialize two
            # debug images every frame unless an external viewer is requested.
            if self.publish_debug_images:
                self._publish_image(message, mask, "mono8")
                self._publish_image(message, overlay, "bgr8")
            if self.preview_enabled:
                now = time.monotonic()
                if now >= self._next_preview_at:
                    cv2.imshow(self.preview_window_name, overlay)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(" "):
                        self._emergency_stop_pub.publish(Bool(data=True))
                        self.get_logger().warn(
                            "SPACE pressed in preview: emergency stop requested."
                        )
                    self._next_preview_at = now + 1.0 / self.preview_fps
            self._model_pub.publish(Float32MultiArray(data=[
                float(confidence), float(lateral_error), float(lookahead_offset),
                float(curvature), float(left_detected), float(right_detected),
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
