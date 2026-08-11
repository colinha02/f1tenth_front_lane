#!/usr/bin/env python3
"""Offline check for the simple grayscale + sliding-window lane tracker."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def seed(histogram: np.ndarray, left: bool) -> int | None:
    mid = histogram.size // 2
    section = histogram[:mid] if left else histogram[mid:]
    return (int(np.argmax(section)) + (0 if left else mid)
            if section.size and int(section.max()) else None)


def track(mask: np.ndarray, seed_x: int | None, seed_y: int, top: int, margin: int = 90) -> np.ndarray:
    if seed_x is None:
        return np.empty((0, 2), dtype=np.int32)
    bottom, count = mask.shape[0], 20
    step, points = max(8, (bottom - top) // count), []
    for direction in (-1, 1):
        x, previous, misses, y = float(seed_x), float(seed_x), 0, seed_y
        while top <= y < bottom:
            y0, y1 = ((max(top, y - step), y) if direction < 0 else (y, min(bottom, y + step)))
            labels, _, stats, _ = cv2.connectedComponentsWithStats(
                mask[y0:y1, :], connectivity=8
            )
            candidates = []
            for label in range(1, labels):
                area = int(stats[label, cv2.CC_STAT_AREA])
                width = int(stats[label, cv2.CC_STAT_WIDTH])
                if area < 20 or area > 700 or width > 70:
                    continue
                x_next = float(stats[label, cv2.CC_STAT_LEFT]) + 0.5 * width
                if abs(x_next - x) <= margin:
                    y_next = float(stats[label, cv2.CC_STAT_TOP] + y0) + 0.5 * float(stats[label, cv2.CC_STAT_HEIGHT])
                    candidates.append((x_next, y_next))
            if candidates:
                x_next, y_next = min(candidates, key=lambda item: abs(item[0] - x))
                points.append((int(round(x_next)), int(round(y_next))))
                previous, x, misses = x, x_next, 0
            else:
                misses += 1
                if misses > 2:
                    break
                x += np.clip(x - previous, -margin, margin)
            y += direction * step
    return np.asarray(points, dtype=np.int32) if points else np.empty((0, 2), dtype=np.int32)


def render(image: np.ndarray, threshold: int) -> np.ndarray:
    h, w = image.shape[:2]
    top, gray = h // 2, cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    mask[:top] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    seed_y, band = int(0.75 * h), 12
    histogram = np.sum(mask[seed_y - band:seed_y + band + 1] > 0, axis=0)
    left, right = track(mask, seed(histogram, True), seed_y, top), track(mask, seed(histogram, False), seed_y, top)
    overlay = image.copy()
    cv2.line(overlay, (0, top), (w - 1, top), (0, 165, 255), 2)
    for points, color in ((left, (255, 0, 0)), (right, (0, 0, 255))):
        if len(points) >= 2:
            points = points[np.argsort(points[:, 1])]
            cv2.polylines(overlay, [points], False, color, 5, cv2.LINE_AA)
            for point in points:
                cv2.circle(overlay, tuple(point), 5, color, -1)
    return cv2.hconcat((image, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), overlay))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--threshold", type=int, default=180)
    parser.add_argument("--save", type=Path)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"Cannot read image: {args.image}")
    result = render(image, args.threshold)
    if not args.no_gui:
        cv2.imshow("Offline simple lane check", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.save), result)


if __name__ == "__main__":
    main()
