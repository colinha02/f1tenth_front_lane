#!/usr/bin/env python3
"""Offline visual check for the front-camera white-lane algorithm.

This script has no ROS, OAK, or vehicle-control dependency.  Give it a saved
front-camera image (preferably a screenshot of the IMU-stabilized view) to
inspect the candidate mask and the two fitted lane curves.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def candidate_mask(
    bgr: np.ndarray,
    top: int,
    bottom: int,
    lightness_min: int,
    saturation_max: int,
    dark_lightness_max: int,
    dark_adjacency_px: int,
) -> np.ndarray:
    hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    white = cv2.inRange(
        hls,
        np.array([0, lightness_min, 0], dtype=np.uint8),
        np.array([179, 255, saturation_max], dtype=np.uint8),
    )
    dark = cv2.inRange(
        hls,
        np.array([0, 0, 0], dtype=np.uint8),
        np.array([179, dark_lightness_max, 255], dtype=np.uint8),
    )
    near_dark = cv2.dilate(
        dark, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (max(1, dark_adjacency_px | 1),) * 2
        )
    )
    mask = cv2.bitwise_and(white, near_dark)
    roi_mask = np.zeros_like(mask)
    roi_mask[top:bottom, :] = 255
    mask = cv2.bitwise_and(mask, roi_mask)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    return cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
    )


def seed(histogram: np.ndarray, left: bool) -> int | None:
    midpoint = histogram.size // 2
    section = histogram[:midpoint] if left else histogram[midpoint:]
    if section.size == 0 or int(section.max()) == 0:
        return None
    return int(np.argmax(section)) + (0 if left else midpoint)


def sliding_points(
    mask: np.ndarray,
    start_x: int | None,
    top: int,
    bottom: int,
    left_side: bool,
    window_count: int = 12,
    margin: int = 85,
    minimum_window_pixels: int = 35,
) -> np.ndarray:
    window_height = max(1, (bottom - top) // window_count)
    windows: list[list[tuple[np.ndarray, float, float]]] = []
    for index in range(window_count):
        y_high = bottom - index * window_height
        y_low = max(top, y_high - window_height)
        window = mask[y_low:y_high, :]
        labels_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            window, connectivity=8
        )
        components: list[tuple[np.ndarray, float, float]] = []
        for label in range(1, labels_count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < 8:
                continue
            component_y, component_x = np.nonzero(labels == label)
            points = np.column_stack((component_y + y_low, component_x))
            components.append((points, float(centroids[label, 0]), float(area)))
        windows.append(components)
    while windows and not windows[0]:
        windows.pop(0)
    if not windows:
        return np.empty((0, 2), dtype=np.float64)

    midpoint = 0.5 * mask.shape[1]
    initial = [component for component in windows[0]
               if (component[1] < midpoint) == left_side] or windows[0]
    paths = [(np.log1p(area) - (0.02 * abs(x - start_x) if start_x is not None else 0.0),
              [points], [x]) for points, x, area in initial]
    paths = sorted(paths, key=lambda item: item[0], reverse=True)[:16]
    for components in windows[1:]:
        expanded = []
        for score, path_points, centers in paths:
            step = centers[-1] - centers[-2] if len(centers) > 1 else 0.0
            predicted = centers[-1] + np.clip(step, -55, 55)
            for points, x, area in components:
                distance = abs(x - predicted)
                if distance <= 2.0 * margin:
                    expanded.append((score + np.log1p(area) - 0.055 * distance,
                                     path_points + [points], centers + [x]))
        if not expanded:
            break
        paths = sorted(expanded, key=lambda item: item[0], reverse=True)[:16]
    return np.vstack(paths[0][1]).astype(np.float64) if paths else np.empty((0, 2))


def seeded_sliding_points(mask: np.ndarray, seed_x: int | None, reference_y: int, top: int, bottom: int) -> np.ndarray:
    if seed_x is None:
        return np.empty((0, 2), dtype=np.float64)
    nonzero_y, nonzero_x = mask.nonzero()
    selected = []
    for direction in (-1, 1):
        current_x, y = int(seed_x), reference_y
        previous_x, misses = float(current_x), 0
        while top <= y < bottom:
            y0, y1 = (max(top, y - 24), y) if direction < 0 else (y, min(bottom, y + 24))
            inside = ((nonzero_y >= y0) & (nonzero_y < y1) &
                      (nonzero_x >= current_x - 170) & (nonzero_x <= current_x + 170))
            indices = np.flatnonzero(inside)
            if indices.size < 35:
                misses += 1
                if misses > 3:
                    break
                current_x = int(round(current_x + np.clip(current_x - previous_x, -60, 60)))
                y += direction * 24
                continue
            selected.append(np.column_stack((nonzero_y[indices], nonzero_x[indices])))
            previous_x, current_x = float(current_x), int(np.mean(nonzero_x[indices]))
            misses = 0
            y += direction * 24
    return np.vstack(selected).astype(np.float64) if selected else np.empty((0, 2), dtype=np.float64)


def fit_curve(points: np.ndarray, roi_height: int) -> np.ndarray | None:
    if points.shape[0] < 180 or np.ptp(points[:, 0]) < 0.45 * roi_height:
        return None
    return np.polyfit(points[:, 0], points[:, 1], 2)


def curve_points(fit: np.ndarray | None, ys: np.ndarray, width: int) -> np.ndarray | None:
    if fit is None:
        return None
    xs = np.polyval(fit, ys)
    inside = (xs >= 0) & (xs < width)
    if np.count_nonzero(inside) < 2:
        return None
    return np.column_stack((xs[inside], ys[inside])).astype(np.int32)


def skeletonize(mask: np.ndarray) -> np.ndarray:
    work = mask.copy()
    skeleton = np.zeros_like(mask)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while cv2.countNonZero(work) > 0:
        opened = cv2.morphologyEx(work, cv2.MORPH_OPEN, element)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(work, opened))
        work = cv2.erode(work, element)
    return skeleton


def connected_lane_points(skeleton: np.ndarray, seed_x: int | None, reference_y: int) -> np.ndarray:
    if seed_x is None:
        return np.empty((0, 2), dtype=np.float64)
    count, labels, _, _ = cv2.connectedComponentsWithStats(skeleton, connectivity=8)
    best = None
    for label in range(1, count):
        ys, xs = np.nonzero(labels == label)
        if ys.size:
            distance = np.min((xs - seed_x) ** 2 + (ys - reference_y) ** 2)
            if best is None or distance < best[0]:
                best = (distance, ys, xs)
    return (np.column_stack((best[1], best[2])).astype(np.float64)
            if best is not None and best[0] <= 70 * 70 else np.empty((0, 2), dtype=np.float64))


def row_centerline(left: np.ndarray, right: np.ndarray, top: int, bottom: int) -> np.ndarray:
    centers, previous_center, previous_left, previous_right = [], None, None, None
    for y in range(bottom - 1, top - 1, -4):
        def row_x(points):
            values = points[np.abs(points[:, 0] - y) <= 2, 1] if points.size else []
            return float(np.median(values)) if len(values) else None
        lx, rx = row_x(left), row_x(right)
        if lx is not None and rx is not None:
            center = 0.5 * (lx + rx)
        elif lx is not None and previous_center is not None and previous_left is not None:
            center = previous_center + lx - previous_left
        elif rx is not None and previous_center is not None and previous_right is not None:
            center = previous_center + rx - previous_right
        else:
            center = previous_center
        if center is not None: centers.append((int(round(center)), y))
        if lx is not None: previous_left = lx
        if rx is not None: previous_right = rx
        previous_center = center
    return np.asarray(centers, dtype=np.int32)


def displayed_lane_skeleton(skeleton: np.ndarray, top: int, bottom: int) -> np.ndarray:
    labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        skeleton, connectivity=8
    )
    kept = np.zeros_like(skeleton)
    minimum_span = 0.20 * (bottom - top)
    for label in range(1, labels_count):
        y = stats[label, cv2.CC_STAT_TOP]
        height = stats[label, cv2.CC_STAT_HEIGHT]
        if height >= minimum_span and y + height - 1 >= top + 0.45 * (bottom - top):
            kept[labels == label] = 255
    return kept


def render(image: np.ndarray, lightness: int, saturation: int, dark_max: int, dark_adjacency: int) -> np.ndarray:
    height, width = image.shape[:2]
    top = height // 2
    bottom = height
    mask = candidate_mask(
        image, top, bottom, lightness, saturation, dark_max, dark_adjacency
    )
    reference_row = int(np.clip(0.75 * height, top, bottom - 1))
    histogram = np.sum(mask[max(top, reference_row - 8):min(bottom, reference_row + 9)] > 0, axis=0)
    raw_skeleton = skeletonize(mask)
    left_points = connected_lane_points(raw_skeleton, seed(histogram, True), reference_row)
    right_points = connected_lane_points(raw_skeleton, seed(histogram, False), reference_row)
    centers = row_centerline(left_points, right_points, top, bottom)
    ys = np.linspace(bottom - 1, top, 80)
    left_fit = fit_curve(left_points, bottom - top)
    right_fit = fit_curve(right_points, bottom - top)
    width_fit = None
    if left_fit is not None and right_fit is not None:
        candidate_width = right_fit - left_fit
        widths = np.polyval(candidate_width, ys)
        if np.all((widths > 100.0) & (widths < 1400.0)):
            width_fit = candidate_width
    if width_fit is None:
        width_fit = np.array([0.0, 0.0, 720.0])

    left_inferred = False
    right_inferred = False
    if left_fit is None and right_fit is not None:
        left_fit = right_fit - width_fit
        left_inferred = True
    elif right_fit is None and left_fit is not None:
        right_fit = left_fit + width_fit
        right_inferred = True

    overlay = image.copy()
    tracked = np.zeros_like(mask)
    for points in (left_points, right_points):
        if points.size:
            tracked[points[:, 0].astype(np.int32), points[:, 1].astype(np.int32)] = 255
    tracked_trace = cv2.dilate(
        tracked, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    )
    overlay[tracked_trace > 0] = (0, 0, 255)
    if centers.size:
        cv2.polylines(overlay, [centers], False, (0, 255, 0), 3)
        for point in centers:
            cv2.circle(overlay, tuple(point), 3, (0, 255, 0), -1)
    skeleton = displayed_lane_skeleton(skeletonize(mask), top, bottom)
    trace = cv2.dilate(
        skeleton, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    )
    overlay[trace > 0] = (0, 0, 180)
    overlay[skeleton > 0] = (0, 255, 255)
    cv2.line(overlay, (0, top), (width - 1, top), (0, 165, 255), 2)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(mask_bgr, "candidate mask", (15, 35), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 255), 2, cv2.LINE_AA)
    return cv2.hconcat((image, mask_bgr, overlay))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="front-camera image file")
    parser.add_argument("--save", type=Path, help="write the comparison image here")
    parser.add_argument("--no-gui", action="store_true", help="do not open a window")
    args = parser.parse_args()

    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"Cannot read image: {args.image}")

    values = {"lightness": 82, "saturation": 179, "dark max": 42, "dark range": 75}
    window = "Offline front-lane check"

    def current() -> np.ndarray:
        return render(
            image,
            cv2.getTrackbarPos("lightness", window) if not args.no_gui else values["lightness"],
            cv2.getTrackbarPos("saturation", window) if not args.no_gui else values["saturation"],
            cv2.getTrackbarPos("dark max", window) if not args.no_gui else values["dark max"],
            cv2.getTrackbarPos("dark range", window) if not args.no_gui else values["dark range"],
        )

    if args.no_gui:
        result = current()
    else:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("lightness", window, values["lightness"], 255, lambda _: None)
        cv2.createTrackbar("saturation", window, values["saturation"], 255, lambda _: None)
        cv2.createTrackbar("dark max", window, values["dark max"], 255, lambda _: None)
        cv2.createTrackbar("dark range", window, values["dark range"], 101, lambda _: None)
        while True:
            result = current()
            cv2.imshow(window, result)
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
        cv2.destroyAllWindows()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.save), result):
            raise SystemExit(f"Cannot write image: {args.save}")
        print(f"Saved: {args.save}")


if __name__ == "__main__":
    main()
