"""Debug overlay for the detection result (/vision/debug). Pure OpenCV, no ROS.

Shows the camera frame and the robot frame in the principal point, the nut with its
width across flats, d and the offset the robot would have to move, or "NO NUT" with
the reason, and an info line with method, frame rate and source of d.
"""
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

from sa_vision.detection import BACKGROUND_REFERENCE, NutDetection, failure_reason

# BGR colours
CAMERA_AXES_COLOUR = (255, 140, 0)    # blue
ROBOT_AXES_COLOUR = (0, 140, 255)     # orange
ROI_COLOUR = (255, 128, 0)
REJECTED_COLOUR = (160, 160, 160)
NUT_COLOUR = (0, 200, 0)              # green
AMBIGUOUS_COLOUR = (0, 220, 255)      # yellow: passed all filters, but not the only one
HOLE_COLOUR = (255, 0, 255)
CENTRE_COLOUR = (0, 0, 255)
FAILURE_COLOUR = (0, 0, 255)          # red
TEXT_COLOUR = (0, 0, 0)
TEXT_BACKGROUND = (255, 255, 255)
# Geometry of the overlay in px
LINE = 2
AXIS_LENGTH = 90
AXIS_TIP = 0.15                       # arrow head length / arrow length
AXIS_LABEL_GAP = 8
MARKER_SIZE = 30
LABEL_GAP = 12                        # between the nut and its text block
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
REJECTION_FONT_SCALE = 0.5
FAILURE_FONT_SCALE = 1.6
TEXT_MARGIN = 10
TEXT_LINE_HEIGHT = 28
FAILURE_LINE_HEIGHT = 52


def _text(image: np.ndarray, lines: Sequence[str], origin: Tuple[int, int],
          colour=TEXT_COLOUR, scale: float = FONT_SCALE, line_height: int = TEXT_LINE_HEIGHT,
          thickness: int = LINE - 1) -> None:
    """Text lines with a white outline, origin = baseline of the first line."""
    for i, text in enumerate(lines):
        org = (int(origin[0]), int(origin[1] + i * line_height))
        cv2.putText(image, text, org, FONT, scale, TEXT_BACKGROUND, thickness + 3, cv2.LINE_AA)
        cv2.putText(image, text, org, FONT, scale, colour, thickness, cv2.LINE_AA)


def _text_width(lines: Sequence[str], scale: float = FONT_SCALE) -> int:
    return max(cv2.getTextSize(text, FONT, scale, LINE)[0][0] for text in lines)


def _axis(image: np.ndarray, origin: Tuple[int, int], direction: Tuple[int, int], label: str,
          colour) -> None:
    tip = (origin[0] + direction[0] * AXIS_LENGTH, origin[1] + direction[1] * AXIS_LENGTH)
    cv2.arrowedLine(image, origin, tip, colour, LINE, cv2.LINE_AA, tipLength=AXIS_TIP)
    width, height = cv2.getTextSize(label, FONT, FONT_SCALE, LINE)[0]
    # Label beyond the tip, centred on the axis
    x = tip[0] + direction[0] * AXIS_LABEL_GAP - (width // 2 if direction[0] == 0 else 0)
    x -= width if direction[0] < 0 else 0
    y = tip[1] + direction[1] * AXIS_LABEL_GAP + (height // 2 if direction[1] == 0 else 0)
    y += height if direction[1] > 0 else 0
    _text(image, [label], (x, y), colour)


def draw_frames(image: np.ndarray, principal_point: Tuple[float, float]) -> None:
    """Camera frame (x right, y down) and robot frame (x_robo = -y_cam, y_robo = -x_cam)."""
    origin = tuple(int(round(c)) for c in principal_point)
    _axis(image, origin, (1, 0), "x_cam", CAMERA_AXES_COLOUR)
    _axis(image, origin, (0, 1), "y_cam", CAMERA_AXES_COLOUR)
    _axis(image, origin, (0, -1), "x_robo", ROBOT_AXES_COLOUR)
    _axis(image, origin, (-1, 0), "y_robo", ROBOT_AXES_COLOUR)


def info_line(detection: NutDetection, fps: Optional[float]) -> str:
    """Method, frame rate and source of d, e.g. "A' (reference) | 14.9 fps | d = 300 mm"."""
    method = ("A' (reference image)" if detection.background == BACKGROUND_REFERENCE
              else "A (local median)")
    rate = f"{fps:.1f} fps" if fps is not None else "offline"
    distance = ("d estimated from size" if detection.distance_from_size
                else f"d = {detection.distance_mm:.0f} mm (yaml)")
    return f"{method} | {rate} | {distance}"


def nut_label(detection: NutDetection,
              robot_offset_mm: Optional[Tuple[float, float]]) -> Sequence[str]:
    """Text next to a detected nut: s, d and the offset in the robot frame."""
    if detection.distance_from_size:
        lines = [f"s = {detection.across_flats_mm:.1f} mm (assumed)",
                 f"d = {detection.distance_mm:.0f} mm (from size)"]
    else:
        lines = [f"s = {detection.across_flats_mm:.1f} mm {detection.size}".rstrip(),
                 f"d = {detection.distance_mm:.0f} mm"]
    if robot_offset_mm is not None:
        lines += [f"dx_robo = {robot_offset_mm[0]:+.1f} mm",
                  f"dy_robo = {robot_offset_mm[1]:+.1f} mm"]
    return lines


def draw_debug(image_bgr: np.ndarray, detection: NutDetection,
               principal_point: Tuple[float, float], fps: Optional[float] = None,
               robot_offset_mm: Optional[Tuple[float, float]] = None) -> np.ndarray:
    """Copy of the image with frames, candidates, the nut or the failure reason.

    robot_offset_mm: offset of the nut from the optical axis in the robot frame
    (dx_robo, dy_robo), i.e. the distance the robot would have to move.
    """
    out = image_bgr.copy()
    height, width = out.shape[:2]
    x, y, w, h = detection.roi
    if (w, h) != (width, height):
        cv2.rectangle(out, (x, y), (x + w - 1, y + h - 1), ROI_COLOUR, LINE)

    for candidate in detection.candidates:
        if candidate.accepted:
            continue
        cv2.drawContours(out, [candidate.outer], -1, REJECTED_COLOUR, 1)
        if candidate.rejected != "size":  # size rejections are mostly specks
            bx, by, _, _ = cv2.boundingRect(candidate.outer)
            _text(out, [candidate.rejected], (bx, by - 4), REJECTED_COLOUR,
                  REJECTION_FONT_SCALE, thickness=1)

    draw_frames(out, principal_point)
    origin = tuple(int(round(c)) for c in principal_point)

    if detection.valid:
        accepted = [c for c in detection.candidates if c.accepted]
        for candidate in accepted:
            cv2.drawContours(out, [candidate.outer], -1, NUT_COLOUR, LINE)
            cv2.drawContours(out, [candidate.hole], -1, HOLE_COLOUR, 1)
        centre = (int(round(detection.u)), int(round(detection.v)))
        cv2.line(out, origin, centre, ROBOT_AXES_COLOUR, 1, cv2.LINE_AA)
        cv2.drawMarker(out, centre, CENTRE_COLOUR, cv2.MARKER_CROSS, MARKER_SIZE, LINE)
        lines = nut_label(detection, robot_offset_mm)
        bx, by, bw, bh = cv2.boundingRect(accepted[0].outer)
        text_width = _text_width(lines)
        tx = bx + bw + LABEL_GAP
        if tx + text_width > width - TEXT_MARGIN:  # no room on the right: left of the nut
            tx = max(bx - LABEL_GAP - text_width, TEXT_MARGIN)
        block_height = TEXT_LINE_HEIGHT * len(lines)
        ty = min(max(by + TEXT_LINE_HEIGHT, 2 * TEXT_LINE_HEIGHT + TEXT_MARGIN),
                 height - block_height - TEXT_MARGIN)
        _text(out, lines, (tx, ty))
    else:
        for candidate in detection.candidates:
            if candidate.accepted:
                cv2.drawContours(out, [candidate.outer], -1, AMBIGUOUS_COLOUR, LINE)
        top = TEXT_MARGIN + TEXT_LINE_HEIGHT + FAILURE_LINE_HEIGHT
        _text(out, ["NO NUT"], (TEXT_MARGIN, top), FAILURE_COLOUR, FAILURE_FONT_SCALE,
              thickness=LINE + 1)
        _text(out, [failure_reason(detection)], (TEXT_MARGIN, top + TEXT_LINE_HEIGHT + 8),
              FAILURE_COLOUR, thickness=LINE)

    _text(out, [info_line(detection, fps)], (TEXT_MARGIN, TEXT_MARGIN + TEXT_LINE_HEIGHT - 8))
    return out
