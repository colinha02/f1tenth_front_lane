#!/usr/bin/env python3
"""Experimental front-lane tracker with curve-only rotated search windows.

The stable ``front_lane_detector`` remains untouched.  This node keeps its
ordinary horizontal-band windows on a straight, then switches each boundary to
an oriented search rectangle only after its local tangent has *changed*
consistently.  A fixed perspective slope alone is therefore not considered a
curve.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy

from auto_control.front_lane_detector import FrontLaneDetector
from sensor_msgs.msg import Image


class RotatedWindowFrontLaneDetector(FrontLaneDetector):
    """Base lane detector with a curve-only oriented sliding-window tracker."""

    def __init__(self) -> None:
        super().__init__()
        for name, value in {
            # A curve is a change in tangent, not a fixed camera-perspective
            # slope.  Enter/exit thresholds form hysteresis.
            "rotated_turn_enter_deg": 12.0,
            "rotated_turn_exit_deg": 5.0,
            "rotated_turn_history_windows": 3,
            "rotated_window_length_px": 28,
            "rotated_window_width_px": 180,
            "rotated_step_px": 14,
            "rotated_turn_side_extra_px": 35,
            # These are the safeguards used by the BEV tracker.  They make
            # the local direction follow a curve gradually instead of making
            # a single noisy window rotate the tracker abruptly.
            "rotated_max_turn_deg_per_window": 18.0,
            "rotated_max_turn_change_deg_per_window": 10.0,
            "rotated_heading_update_gain": 0.45,
            "show_rotated_window_diagnostics": True,
            "rotated_diagnostic_window_name": "Rotated sliding-window diagnostic",
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.rotated_turn_enter_rad = math.radians(
            max(1.0, float(value("rotated_turn_enter_deg")))
        )
        self.rotated_turn_exit_rad = math.radians(
            max(0.1, float(value("rotated_turn_exit_deg")))
        )
        self.rotated_turn_history_windows = max(
            2, int(value("rotated_turn_history_windows"))
        )
        self.rotated_window_length_px = max(8, int(value("rotated_window_length_px")))
        self.rotated_window_width_px = max(20, int(value("rotated_window_width_px")))
        self.rotated_step_px = max(4, int(value("rotated_step_px")))
        self.rotated_turn_side_extra_px = max(
            0, int(value("rotated_turn_side_extra_px"))
        )
        self.rotated_max_turn_rad = math.radians(
            max(1.0, float(value("rotated_max_turn_deg_per_window")))
        )
        self.rotated_max_turn_change_rad = math.radians(
            max(0.5, float(value("rotated_max_turn_change_deg_per_window")))
        )
        self.rotated_heading_update_gain = float(np.clip(
            float(value("rotated_heading_update_gain")), 0.05, 1.0
        ))
        self.show_rotated_window_diagnostics = bool(
            value("show_rotated_window_diagnostics")
        )
        self.rotated_diagnostic_window_name = str(
            value("rotated_diagnostic_window_name")
        )
        self._rotated_polygons: list[tuple[np.ndarray, bool, bool]] = []
        self._diagnostic_left = np.empty((0, 2), dtype=np.int32)
        self._diagnostic_right = np.empty((0, 2), dtype=np.int32)
        self.get_logger().info(
            "Experimental tracker ready: straight windows + tangent-change rotated windows."
        )

    @staticmethod
    def _unit(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        return vector / norm if norm > 1e-6 else fallback.copy()

    @staticmethod
    def _tangent_change(first: np.ndarray, second: np.ndarray) -> float:
        dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
        return float(math.acos(dot))

    @staticmethod
    def _signed_turn(first: np.ndarray, second: np.ndarray) -> float:
        """Signed image-plane rotation from ``first`` to ``second``."""
        cross = float(first[0] * second[1] - first[1] * second[0])
        dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
        return float(math.atan2(cross, dot))

    @staticmethod
    def _rotate(vector: np.ndarray, angle: float) -> np.ndarray:
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return np.array(
            (
                cosine * vector[0] - sine * vector[1],
                sine * vector[0] + cosine * vector[1],
            ),
            dtype=np.float32,
        )

    def _find_region_candidate(
        self,
        mask: np.ndarray,
        polygon: np.ndarray,
        predicted: np.ndarray,
        tangent: np.ndarray,
        normal: np.ndarray,
    ) -> tuple[float, float] | None:
        """Return the connected white stripe closest to the predicted path."""
        height, width = mask.shape[:2]
        x0 = max(0, int(np.floor(np.min(polygon[:, 0]))))
        x1 = min(width, int(np.ceil(np.max(polygon[:, 0]))) + 1)
        y0 = max(0, int(np.floor(np.min(polygon[:, 1]))))
        y1 = min(height, int(np.ceil(np.max(polygon[:, 1])) + 1))
        if x1 <= x0 or y1 <= y0:
            return None

        local_polygon = np.round(polygon - np.array([x0, y0])).astype(np.int32)
        region = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        cv2.fillConvexPoly(region, local_polygon, 255)
        search = cv2.bitwise_and(mask[y0:y1, x0:x1], region)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(search, connectivity=8)
        candidates: list[tuple[float, float, float]] = []
        max_area = max(self.maximum_component_pixels * 5, 3000)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.minimum_window_pixels or area > max_area:
                continue
            ys, xs = np.nonzero(labels == label)
            if xs.size < self.minimum_window_pixels:
                continue
            coordinates = np.column_stack((xs + x0, ys + y0)).astype(np.float32)
            # This is the actual centre of the detected white stripe inside
            # the oriented window, equivalent to the response centroid used
            # by the BEV implementation.  The mask is binary, so its mean is
            # the weighted centroid with equal white-pixel weights.
            point = np.mean(coordinates, axis=0)
            offset = point - predicted
            normal_error = abs(float(np.dot(offset, normal)))
            longitudinal_error = abs(float(np.dot(offset, tangent)))
            # Normal closeness keeps us on the predicted boundary; a small
            # longitudinal term prefers the next section, not a remote blob.
            score = normal_error + 0.20 * longitudinal_error
            candidates.append((score, float(point[0]), float(point[1])))
        if not candidates:
            return None
        _, x, y = min(candidates, key=lambda item: item[0])
        return x, y

    @staticmethod
    def _axis_polygon(x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
        return np.array(((x0, y0), (x1, y0), (x1, y1), (x0, y1)), dtype=np.float32)

    def _rotated_polygon(
        self,
        center: np.ndarray,
        tangent: np.ndarray,
        widen_on_positive_normal: bool,
    ) -> np.ndarray:
        normal = np.array((-tangent[1], tangent[0]), dtype=np.float32)
        half_length = 0.5 * self.rotated_window_length_px
        half_width = 0.5 * self.rotated_window_width_px
        extra = float(self.rotated_turn_side_extra_px)
        positive = half_width + (extra if widen_on_positive_normal else 0.0)
        negative = half_width + (0.0 if widen_on_positive_normal else extra)
        return np.asarray((
            center - tangent * half_length - normal * negative,
            center + tangent * half_length - normal * negative,
            center + tangent * half_length + normal * positive,
            center - tangent * half_length + normal * positive,
        ), dtype=np.float32)

    def _track_direction(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int, direction: int,
    ) -> tuple[list[tuple[int, int]], list[tuple[int, int, int, int, bool]]]:
        if seed_x is None:
            return [], []

        height, width = mask.shape[:2]
        base_height = max(8, (bottom - top) // self.window_count)
        last = np.array((float(seed_x), float(seed_y)), dtype=np.float32)
        tangent = np.array((0.0, float(direction)), dtype=np.float32)
        tangent_history: list[np.ndarray] = []
        curve_mode = False
        straight_windows = 0
        misses = 0
        previous_applied_turn = 0.0
        points: list[tuple[int, int]] = []
        windows: list[tuple[int, int, int, int, bool]] = []

        # Curves can become close to horizontal in the image.  A y-only loop
        # would stop there, so the oriented mode is bounded by steps instead.
        max_steps = max(self.window_count * 5, 30)
        for _ in range(max_steps):
            if not (0 <= last[0] < width and top <= last[1] < bottom):
                break
            normal = np.array((-tangent[1], tangent[0]), dtype=np.float32)
            # The forward/upward arm is the only one used for predicting the
            # road ahead.  The short downward arm is retained for the
            # existing centre/width logic, but never enters rotated mode.
            use_rotated_window = curve_mode and direction < 0
            if use_rotated_window:
                predicted = last + tangent * float(self.rotated_step_px)
                polygon = self._rotated_polygon(
                    predicted, tangent, widen_on_positive_normal=(tangent[0] > 0.0)
                )
                step_is_curve = True
            else:
                next_y = last[1] + direction * base_height
                if next_y < top or next_y >= bottom:
                    break
                # Retain the original tracker behaviour on a straight: its
                # fixed perspective tilt is predicted, but does not activate
                # rotated windows by itself.
                slope = tangent[0] / tangent[1] if abs(float(tangent[1])) > 1e-3 else 0.0
                predicted = np.array(
                    (last[0] + slope * (next_y - last[1]), next_y), dtype=np.float32
                )
                polygon = self._axis_polygon(
                    predicted[0] - self.window_margin_px,
                    predicted[0] + self.window_margin_px,
                    min(last[1], next_y), max(last[1], next_y),
                )
                step_is_curve = False

            candidate = self._find_region_candidate(
                mask, polygon, predicted, tangent, normal
            )
            clipped_polygon = polygon.copy()
            clipped_polygon[:, 0] = np.clip(clipped_polygon[:, 0], 0, width - 1)
            clipped_polygon[:, 1] = np.clip(clipped_polygon[:, 1], top, bottom - 1)
            self._rotated_polygons.append((clipped_polygon, candidate is not None, step_is_curve))
            x0 = int(np.floor(np.min(clipped_polygon[:, 0])))
            x1 = int(np.ceil(np.max(clipped_polygon[:, 0])))
            y0 = int(np.floor(np.min(clipped_polygon[:, 1])))
            y1 = int(np.ceil(np.max(clipped_polygon[:, 1])))
            windows.append((x0, x1, y0, y1, candidate is not None))

            if candidate is None:
                misses += 1
                if misses > self.maximum_missing_windows:
                    break
                # Preserve the estimated tangent during a short gap.
                last = predicted
                continue

            next_point = np.array(candidate, dtype=np.float32)
            movement = next_point - last
            measured = self._unit(movement, tangent)
            # Never let a noisy candidate reverse the intended travel axis.
            if measured[1] * direction < -0.05:
                measured = tangent.copy()

            # Core BEV behaviour: calculate the observed turn from the
            # white-stripe centroid, clamp both the turn and its frame-to-
            # frame change, then rotate the predicted direction gradually.
            # On a straight the normal windows remain in use; this direction
            # estimate simply preserves the existing tangent prediction.
            desired_turn = float(np.clip(
                self._signed_turn(tangent, measured),
                -self.rotated_max_turn_rad,
                self.rotated_max_turn_rad,
            ))
            desired_turn = float(np.clip(
                desired_turn,
                previous_applied_turn - self.rotated_max_turn_change_rad,
                previous_applied_turn + self.rotated_max_turn_change_rad,
            ))
            applied_turn = previous_applied_turn + self.rotated_heading_update_gain * (
                desired_turn - previous_applied_turn
            )
            tangent = self._unit(self._rotate(tangent, applied_turn), tangent)
            previous_applied_turn = applied_turn
            tangent_history.append(tangent.copy())
            if len(tangent_history) > self.rotated_turn_history_windows:
                tangent_history.pop(0)
            turn = (
                self._tangent_change(tangent_history[0], tangent_history[-1])
                if len(tangent_history) >= self.rotated_turn_history_windows else 0.0
            )
            if (
                direction < 0
                and not curve_mode
                and turn >= self.rotated_turn_enter_rad
            ):
                curve_mode = True
                straight_windows = 0
            elif curve_mode:
                if turn <= self.rotated_turn_exit_rad:
                    straight_windows += 1
                    if straight_windows >= self.rotated_turn_history_windows:
                        curve_mode = False
                        straight_windows = 0
                else:
                    straight_windows = 0
            points.append((int(round(next_point[0])), int(round(next_point[1]))))
            last = next_point
            misses = 0
        return points, windows

    def _track_lane(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int,
    ) -> tuple[np.ndarray, list[tuple[int, int, int, int, bool]]]:
        # Do not call the base implementation here: it only returns ordinary
        # horizontal windows.  Keep its two-direction output contract so all
        # existing width profile, virtual-centreline and control code stays
        # exactly unchanged.
        upward, up_windows = self._track_direction(
            mask, seed_x, seed_y, top, bottom, -1
        )
        downward, down_windows = self._track_direction(
            mask, seed_x, seed_y, top, bottom, 1
        )
        points = upward + downward
        points_array = (
            np.asarray(points, dtype=np.int32)
            if points else np.empty((0, 2), dtype=np.int32)
        )
        windows = up_windows + down_windows
        # Retain points solely for the supplementary rotated-window view.
        if seed_x is not None and seed_x < mask.shape[1] // 2:
            self._diagnostic_left = points_array
        else:
            self._diagnostic_right = points_array
        return points_array, windows

    def _on_image(self, message: Image) -> None:
        self._rotated_polygons = []
        self._diagnostic_left = np.empty((0, 2), dtype=np.int32)
        self._diagnostic_right = np.empty((0, 2), dtype=np.int32)
        super()._on_image(message)
        if not (self.preview_enabled and self.show_rotated_window_diagnostics):
            return
        try:
            diagnostic = self._nv12_to_bgr(message)
            for polygon, found, curve in self._rotated_polygons:
                color = (0, 165, 255) if curve else (120, 120, 120)
                if not found:
                    color = (70, 70, 180) if curve else (70, 70, 70)
                cv2.polylines(diagnostic, [np.round(polygon).astype(np.int32)], True, color, 1)
            if self._diagnostic_left.shape[0] > 1:
                cv2.polylines(diagnostic, [self._diagnostic_left], False, (255, 0, 0), 3)
            if self._diagnostic_right.shape[0] > 1:
                cv2.polylines(diagnostic, [self._diagnostic_right], False, (0, 0, 255), 3)
            cv2.putText(
                diagnostic, "gray = straight / orange = rotated curve windows",
                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA,
            )
            cv2.imshow(self.rotated_diagnostic_window_name, diagnostic)
        except Exception as error:
            self.get_logger().warn(f"rotated diagnostic error: {error}")

    def destroy_node(self) -> bool:
        try:
            cv2.destroyWindow(self.rotated_diagnostic_window_name)
        except cv2.error:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RotatedWindowFrontLaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
