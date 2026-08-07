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
    edge_low: int,
    edge_high: int,
) -> np.ndarray:
    hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    white = cv2.inRange(
        hls,
        np.array([0, lightness_min, 0], dtype=np.uint8),
        np.array([179, 255, saturation_max], dtype=np.uint8),
    )
    edges = cv2.Canny(gray, edge_low, edge_high)
    edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    mask = cv2.bitwise_and(white, edges)
    roi_mask = np.zeros_like(mask)
    roi_mask[top:bottom, :] = 255
    mask = cv2.bitwise_and(mask, roi_mask)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
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
            if area < minimum_window_pixels:
                continue
            component_y, component_x = np.nonzero(labels == label)
            points = np.column_stack((component_y + y_low, component_x))
            components.append((points, float(centroids[label, 0]), float(area)))
        windows.append(components)
    if not windows or not windows[0]:
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


def render(image: np.ndarray, lightness: int, saturation: int, edge_low: int, edge_high: int) -> np.ndarray:
    height, width = image.shape[:2]
    top = height // 2
    bottom = height
    mask = candidate_mask(
        image, top, bottom, lightness, saturation, edge_low, edge_high
    )
    histogram = np.sum(mask[max(top, bottom - 80):bottom] > 0, axis=0)
    ys = np.linspace(bottom - 1, top, 80)
    left_fit = fit_curve(
        sliding_points(mask, seed(histogram, True), top, bottom, True), bottom - top
    )
    right_fit = fit_curve(
        sliding_points(mask, seed(histogram, False), top, bottom, False), bottom - top
    )

    overlay = image.copy()
    cv2.line(overlay, (0, top), (width - 1, top), (0, 165, 255), 2)
    for fit in (left_fit, right_fit):
        points = curve_points(fit, ys, width)
        if points is not None:
            cv2.polylines(overlay, [points], False, (255, 0, 0), 5)
    if left_fit is not None and right_fit is not None:
        center = curve_points(0.5 * (left_fit + right_fit), ys, width)
        if center is not None:
            cv2.polylines(overlay, [center], False, (0, 255, 0), 3)

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

    values = {"lightness": 165, "saturation": 120, "edge low": 35, "edge high": 110}
    window = "Offline front-lane check"

    def current() -> np.ndarray:
        return render(
            image,
            cv2.getTrackbarPos("lightness", window) if not args.no_gui else values["lightness"],
            cv2.getTrackbarPos("saturation", window) if not args.no_gui else values["saturation"],
            cv2.getTrackbarPos("edge low", window) if not args.no_gui else values["edge low"],
            cv2.getTrackbarPos("edge high", window) if not args.no_gui else values["edge high"],
        )

    if args.no_gui:
        result = current()
    else:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("lightness", window, values["lightness"], 255, lambda _: None)
        cv2.createTrackbar("saturation", window, values["saturation"], 255, lambda _: None)
        cv2.createTrackbar("edge low", window, values["edge low"], 255, lambda _: None)
        cv2.createTrackbar("edge high", window, values["edge high"], 255, lambda _: None)
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
