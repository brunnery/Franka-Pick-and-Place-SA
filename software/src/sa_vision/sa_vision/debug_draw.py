"""Debug overlay for the detection result. Pure OpenCV, no ROS."""
from typing import Tuple

import cv2
import numpy as np

from sa_vision.detection import NutDetection

# BGR colours and line widths of the overlay
ROI_COLOUR = (255, 128, 0)
REJECTED_COLOUR = (160, 160, 160)
OUTER_COLOUR = (0, 200, 0)
HOLE_COLOUR = (255, 0, 255)
CENTRE_COLOUR = (0, 0, 255)
PRINCIPAL_POINT_COLOUR = (0, 220, 255)
TEXT_COLOUR = (0, 0, 0)
TEXT_BACKGROUND = (255, 255, 255)
LINE = 2
MARKER_SIZE = 30
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
TEXT_ORIGIN = (10, 30)
TEXT_LINE_HEIGHT = 28


def _text_lines(image: np.ndarray, lines) -> None:
    for i, text in enumerate(lines):
        org = (TEXT_ORIGIN[0], TEXT_ORIGIN[1] + i * TEXT_LINE_HEIGHT)
        cv2.putText(image, text, org, FONT, FONT_SCALE, TEXT_BACKGROUND, LINE + 3, cv2.LINE_AA)
        cv2.putText(image, text, org, FONT, FONT_SCALE, TEXT_COLOUR, LINE - 1, cv2.LINE_AA)


def draw_debug(image_bgr: np.ndarray, detection: NutDetection,
               principal_point: Tuple[float, float], label: str = "") -> np.ndarray:
    """Copy of the image with ROI, silhouettes, dark core, nut centre and principal point."""
    out = image_bgr.copy()
    x, y, w, h = detection.roi
    cv2.rectangle(out, (x, y), (x + w - 1, y + h - 1), ROI_COLOUR, LINE)

    for candidate in detection.candidates:
        if candidate.accepted:
            cv2.drawContours(out, [candidate.outer], -1, OUTER_COLOUR, LINE)
            cv2.drawContours(out, [candidate.hole], -1, HOLE_COLOUR, LINE)
        else:
            cv2.drawContours(out, [candidate.outer], -1, REJECTED_COLOUR, 1)

    cx, cy = (int(round(c)) for c in principal_point)
    cv2.drawMarker(out, (cx, cy), PRINCIPAL_POINT_COLOUR, cv2.MARKER_CROSS, 2 * MARKER_SIZE, LINE)

    if detection.valid:
        centre = (int(round(detection.u)), int(round(detection.v)))
        cv2.drawMarker(out, centre, CENTRE_COLOUR, cv2.MARKER_TILTED_CROSS, MARKER_SIZE, LINE)
        cv2.line(out, (cx, cy), centre, PRINCIPAL_POINT_COLOUR, 1, cv2.LINE_AA)

    lines = [f"{detection.status}  (background: {detection.background})"]
    if detection.valid:
        lines.append(f"u={detection.u:.1f} v={detection.v:.1f}  "
                     f"s={detection.across_flats_mm:.1f} mm {detection.size or '(size ?)'}  "
                     f"s/e={detection.width_ratio:.2f}")
    if label:
        lines.append(label)
    _text_lines(out, lines)
    return out
