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
    # Equivalent to the former large elliptical dilation, but far cheaper for
    # real-time use: retain white pixels within the configured radius of the
    # black track.
    distance_to_dark = cv2.distanceTransform(
        cv2.bitwise_not(dark), cv2.DIST_L2, 3
    )
    near_dark = np.where(
        distance_to_dark <= 0.5 * max(1, dark_adjacency_px | 1), 255, 0
    ).astype(np.uint8)
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
    smoothed = cv2.GaussianBlur(
        histogram.astype(np.float32).reshape(1, -1), (31, 1), 0
    ).reshape(-1)
    midpoint = smoothed.size // 2
    section = smoothed[:midpoint] if left else smoothed[midpoint:]
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


def reference_seeded_sliding_points(
    mask: np.ndarray,
    start_x: int | None,
    reference_y: int,
    top: int,
    bottom: int,
    left_side: bool,
    window_count: int = 12,
    margin: int = 85,
) -> np.ndarray:
    """Trace both directions from the histogram seed, not from image bottom."""
    if start_x is None:
        return np.empty((0, 2), dtype=np.float64)
    height = max(1, (bottom - top) // window_count)
    windows = []
    for index in range(window_count):
        y_high = bottom - index * height
        y_low = max(top, y_high - height)
        labels_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask[y_low:y_high, :], connectivity=8
        )
        components = []
        for label in range(1, labels_count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            if area < 8 or area > 1200 or width > 100:
                continue
            ys, xs = np.nonzero(labels == label)
            components.append((
                np.column_stack((ys + y_low, xs)),
                float(centroids[label, 0]),
                float(stats[label, cv2.CC_STAT_AREA]),
            ))
        windows.append(components)
    anchor = next((i for i in range(window_count)
                   if bottom - (i + 1) * height <= reference_y < bottom - i * height),
                  window_count - 1)
    midpoint = 0.5 * mask.shape[1]
    candidates = [item for item in windows[anchor]
                  if (item[1] < midpoint) == left_side and abs(item[1] - start_x) <= max(2 * margin, 130)]
    if not candidates:
        return np.empty((0, 2), dtype=np.float64)
    paths = [(np.log1p(area) - 0.08 * abs(x - start_x), [points], [x], 0)
             for points, x, area in candidates]
    paths = sorted(paths, reverse=True, key=lambda item: item[0])[:16]

    def extend(direction: int):
        active = paths
        for index in range(anchor + direction, len(windows) if direction > 0 else -1, direction):
            expanded = []
            for score, selected, centers, misses in active:
                if not windows[index]:
                    if misses < 2:
                        expanded.append((score - 2.0, selected, centers, misses + 1))
                    continue
                step = centers[-1] - centers[-2] if len(centers) > 1 else 0.0
                predicted = centers[-1] + np.clip(step, -55, 55)
                maximum = max(2.0 * margin, margin + 1.8 * abs(step))
                for points, x, area in windows[index]:
                    distance = abs(x - predicted)
                    if distance <= maximum:
                        expanded.append((score + np.log1p(area) - 0.055 * distance,
                                         selected + [points], centers + [x], 0))
            if not expanded:
                break
            active = sorted(expanded, reverse=True, key=lambda item: item[0])[:16]
        return active[0][1] if active else []

    points = extend(1) + extend(-1)
    return np.vstack(points).astype(np.float64) if points else np.empty((0, 2), dtype=np.float64)


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
    previous_step, previous_half_width, missing_rows = 0.0, None, 0
    for y in range(bottom - 1, top - 1, -4):
        def row_x(points):
            values = points[np.abs(points[:, 0] - y) <= 2, 1] if points.size else []
            return float(np.median(values)) if len(values) else None
        lx, rx = row_x(left), row_x(right)
        if lx is not None and rx is not None:
            center = 0.5 * (lx + rx)
            half_width = 0.5 * (rx - lx)
            if half_width > 15.0:
                previous_half_width = half_width if previous_half_width is None else 0.75 * previous_half_width + 0.25 * half_width
        elif lx is not None and previous_center is not None and previous_left is not None:
            center = lx + previous_half_width if previous_half_width is not None else previous_center + lx - previous_left
        elif rx is not None and previous_center is not None and previous_right is not None:
            center = rx - previous_half_width if previous_half_width is not None else previous_center + rx - previous_right
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


def points_mask(points: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    result = np.zeros(shape, dtype=np.uint8)
    if points.size:
        ys = np.clip(points[:, 0].astype(np.int32), 0, shape[0] - 1)
        xs = np.clip(points[:, 1].astype(np.int32), 0, shape[1] - 1)
        result[ys, xs] = 255
    return result


def tape_edge_centers(gray: np.ndarray, selected: np.ndarray, top: int, bottom: int, threshold: int = 55):
    if cv2.countNonZero(selected) == 0:
        return np.empty((0, 2), dtype=np.float64), np.empty((0, 2), dtype=np.float64)
    gradient = cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3))
    search = cv2.dilate(selected, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    edge_mask = np.where((gradient >= threshold) & (search > 0), 255, 0).astype(np.uint8)
    centers = []
    for y in range(bottom - 1, top - 1, -2):
        tape_x = np.flatnonzero(selected[y])
        if tape_x.size < 2:
            continue
        left_limit, right_limit = int(tape_x.min()), int(tape_x.max())
        edge_x = np.flatnonzero(edge_mask[y])
        left_options = edge_x[np.abs(edge_x - left_limit) <= 5]
        right_options = edge_x[np.abs(edge_x - right_limit) <= 5]
        left_edge = int(left_options[np.argmin(np.abs(left_options - left_limit))]) if left_options.size else left_limit
        right_edge = int(right_options[np.argmin(np.abs(right_options - right_limit))]) if right_options.size else right_limit
        if right_edge - left_edge >= 2:
            centers.append((float(y), 0.5 * (left_edge + right_edge)))
    return np.column_stack(np.nonzero(edge_mask)).astype(np.float64), np.asarray(centers, dtype=np.float64)


def smooth_centerline(centers: np.ndarray, top: int, bottom: int, bin_height: int = 12, outlier_px: float = 24.0) -> np.ndarray:
    if centers.shape[0] < 4:
        return centers.astype(np.int32)
    bins = []
    for y0 in range(top, bottom, bin_height):
        values = centers[(centers[:, 1] >= y0) & (centers[:, 1] < y0 + bin_height)]
        if values.size:
            bins.append((float(np.median(values[:, 0])), float(np.median(values[:, 1]))))
    samples = np.asarray(bins, dtype=np.float64)
    if samples.shape[0] < 4:
        return centers.astype(np.int32)
    degree = 2 if samples.shape[0] >= 6 else 1
    fit = np.polyfit(samples[:, 1], samples[:, 0], degree)
    inliers = np.abs(samples[:, 0] - np.polyval(fit, samples[:, 1])) <= outlier_px
    if np.count_nonzero(inliers) >= degree + 2:
        fit = np.polyfit(samples[inliers, 1], samples[inliers, 0], degree)
        samples = samples[inliers]
    ys = np.arange(int(np.max(samples[:, 1])), int(np.min(samples[:, 1])) - 1, -4, dtype=np.float64)
    return np.column_stack((np.polyval(fit, ys), ys)).astype(np.int32)


def lane_x_near_row(points: np.ndarray, row: int, maximum_distance: int = 180) -> float | None:
    if points.size == 0:
        return None
    distances = np.abs(points[:, 0] - row)
    nearest = float(np.min(distances))
    if nearest > maximum_distance:
        return None
    values = points[distances <= nearest + 4.0, 1]
    return float(np.median(values)) if values.size else None


def render(image: np.ndarray, lightness: int, saturation: int, dark_max: int, dark_adjacency: int) -> np.ndarray:
    height, width = image.shape[:2]
    top = height // 2
    bottom = height
    mask = candidate_mask(
        image, top, bottom, lightness, saturation, dark_max, dark_adjacency
    )
    reference_row = int(np.clip(0.75 * height, top, bottom - 1))
    histogram = np.sum(mask[max(top, reference_row - 8):min(bottom, reference_row + 9)] > 0, axis=0)
    # Use the same continuous sliding-window path selection as the ROS node.
    # Selecting an entire skeleton component here made a curve branch into
    # bright background markings even though a sliding implementation existed.
    left_points = reference_seeded_sliding_points(
        mask, seed(histogram, True), reference_row, top, bottom, True
    )
    right_points = reference_seeded_sliding_points(
        mask, seed(histogram, False), reference_row, top, bottom, False
    )
    left_selected = points_mask(left_points, mask.shape)
    right_selected = points_mask(right_points, mask.shape)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    left_edges, left_centers = tape_edge_centers(gray, left_selected, top, bottom)
    right_edges, right_centers = tape_edge_centers(gray, right_selected, top, bottom)
    left_roi_x = lane_x_near_row(left_centers, top)
    right_roi_x = lane_x_near_row(right_centers, top)
    roi_center_x = 0.5 * (left_roi_x + right_roi_x) if left_roi_x is not None and right_roi_x is not None else None
    ys = np.linspace(bottom - 1, top, 80)
    left_fit = fit_curve(left_centers, bottom - top)
    right_fit = fit_curve(right_centers, bottom - top)
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
    tracked = cv2.bitwise_or(left_selected, right_selected)
    tracked_trace = cv2.dilate(
        tracked, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    )
    overlay[tracked_trace > 0] = (0, 0, 255)
    for edges in (left_edges, right_edges):
        if edges.size:
            overlay[edges[:, 0].astype(np.int32), edges[:, 1].astype(np.int32)] = (255, 255, 0)
    if roi_center_x is not None:
        bottom_anchor = (width // 2, bottom - 14)
        roi_anchor = (int(round(roi_center_x)), top)
        cv2.line(overlay, bottom_anchor, roi_anchor, (0, 255, 0), 4)
        cv2.circle(overlay, bottom_anchor, 8, (0, 255, 0), -1)
        cv2.circle(overlay, roi_anchor, 8, (0, 255, 0), -1)
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

    values = {"lightness": 82, "saturation": 179, "dark max": 42, "dark range": 35}
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
