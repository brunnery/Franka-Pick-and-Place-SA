"""Debug images for the detection result (/vision/debug). Pure OpenCV, no ROS.

draw_debug: info line (method, frame rate, source of d) and camera frame at the top
left, robot frame at the bottom right, the nut with a line from the principal point,
and at the top right its width across flats, d and the offset the robot would have
to move, or "NO NUT" with the reason.
draw_steps: 2x2 mosaic of the intermediate steps (I/B, threshold, morphology with
candidates, result) to find where the detection fails.
"""
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

from sa_vision.detection import BACKGROUND_REFERENCE, NutDetection, NutParams, failure_reason

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
PRINCIPAL_POINT_MARKER_SIZE = 20
FRAME_INSET = 20                      # frame origins: distance from the image corner
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
REJECTION_FONT_SCALE = 0.5
FAILURE_FONT_SCALE = 1.6
TEXT_MARGIN = 10
TEXT_LINE_HEIGHT = 28
FAILURE_LINE_HEIGHT = 52
# Step mosaic
RATIO_DISPLAY_RANGE = (0.5, 1.5)      # I/B shown from black to white, table = mid grey
STEP_BRIGHT_COLOUR = (0, 220, 255)    # yellow: brighter than the table
STEP_DARK_COLOUR = (255, 120, 0)      # blue: darker than the table
STEP_CORE_COLOUR = (0, 0, 255)        # red: dark enough for the core (thread hole)
STEP_MASK_GREY = 90                   # foreground after morphology
STEP_LABEL_MIN_AREA_PX = 300          # rejected candidates smaller than this get no label
STEP_TITLE_FONT_SCALE = 0.8
STEP_TITLES = (
    "1  I/B  (table = grey, brighter = white, darker = black)",
    "2  threshold  (yellow brighter, blue darker, red core)",
    "3  morphology + candidates  (grey = reason)",
    "4  result",
)


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
    x = min(max(x, TEXT_MARGIN), image.shape[1] - TEXT_MARGIN - width)  # stay inside
    _text(image, [label], (x, y), colour)


def draw_frames(image: np.ndarray) -> None:
    """Axis legends: camera frame (x right, y down) at the top left, below the info line,
    robot frame (x_robo = -y_cam, y_robo = -x_cam) at the bottom right."""
    height, width = image.shape[:2]
    camera = (TEXT_MARGIN + FRAME_INSET, TEXT_MARGIN + TEXT_LINE_HEIGHT + FRAME_INSET)
    _axis(image, camera, (1, 0), "x_cam", CAMERA_AXES_COLOUR)
    _axis(image, camera, (0, 1), "y_cam", CAMERA_AXES_COLOUR)
    robot = (width - TEXT_MARGIN - FRAME_INSET, height - TEXT_MARGIN - FRAME_INSET)
    _axis(image, robot, (0, -1), "x_robo", ROBOT_AXES_COLOUR)
    _axis(image, robot, (-1, 0), "y_robo", ROBOT_AXES_COLOUR)


def _text_top_right(image: np.ndarray, lines: Sequence[str], baseline: int, colour,
                    scale: float = FONT_SCALE, line_height: int = TEXT_LINE_HEIGHT,
                    thickness: int = LINE - 1) -> None:
    """Text lines right-aligned at the right image border."""
    width = image.shape[1]
    for i, text in enumerate(lines):
        x = width - TEXT_MARGIN - _text_width([text], scale)
        _text(image, [text], (x, baseline + i * line_height), colour, scale, line_height,
              thickness)


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
    """Text for a detected nut (top right): s, d and the offset in the robot frame."""
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

    draw_frames(out)
    origin = tuple(int(round(c)) for c in principal_point)
    cv2.drawMarker(out, origin, CAMERA_AXES_COLOUR, cv2.MARKER_CROSS,
                   PRINCIPAL_POINT_MARKER_SIZE, 1)
    first_baseline = TEXT_MARGIN + TEXT_LINE_HEIGHT - 8

    if detection.valid:
        accepted = [c for c in detection.candidates if c.accepted]
        for candidate in accepted:
            cv2.drawContours(out, [candidate.outer], -1, NUT_COLOUR, LINE)
            cv2.drawContours(out, [candidate.hole], -1, HOLE_COLOUR, 1)
        centre = (int(round(detection.u)), int(round(detection.v)))
        cv2.line(out, origin, centre, ROBOT_AXES_COLOUR, 1, cv2.LINE_AA)
        cv2.drawMarker(out, centre, CENTRE_COLOUR, cv2.MARKER_CROSS, MARKER_SIZE, LINE)
        _text_top_right(out, nut_label(detection, robot_offset_mm), first_baseline, TEXT_COLOUR)
    else:
        for candidate in detection.candidates:
            if candidate.accepted:
                cv2.drawContours(out, [candidate.outer], -1, AMBIGUOUS_COLOUR, LINE)
        top = TEXT_MARGIN + FAILURE_LINE_HEIGHT - 8
        _text_top_right(out, ["NO NUT"], top, FAILURE_COLOUR, FAILURE_FONT_SCALE,
                        thickness=LINE + 1)
        _text_top_right(out, [failure_reason(detection)], top + TEXT_LINE_HEIGHT + 8,
                        FAILURE_COLOUR, thickness=LINE)

    _text(out, [info_line(detection, fps)], (TEXT_MARGIN, first_baseline))
    return out


def _roi_to_frame(roi_image: np.ndarray, roi: Tuple[int, int, int, int],
                  shape: Tuple[int, int]) -> np.ndarray:
    """BGR image of the full frame with the ROI image at its place, black elsewhere."""
    frame = np.zeros((shape[0], shape[1], 3), np.uint8)
    x, y, w, h = roi
    if roi_image is not None and w > 0 and h > 0:
        frame[y:y + h, x:x + w] = roi_image
    return frame


def ratio_view(detection: NutDetection, shape: Tuple[int, int]) -> np.ndarray:
    """Step 1: I/B as grey levels."""
    if detection.ratio is None:
        return _roi_to_frame(None, detection.roi, shape)
    low, high = RATIO_DISPLAY_RANGE
    grey = np.clip((detection.ratio - low) / (high - low) * 255.0, 0, 255).astype(np.uint8)
    return _roi_to_frame(cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR), detection.roi, shape)


def threshold_view(detection: NutDetection, params: NutParams,
                   shape: Tuple[int, int]) -> np.ndarray:
    """Step 2: pixel classes of the two-sided threshold, before morphology."""
    if detection.ratio is None:
        return _roi_to_frame(None, detection.roi, shape)
    r = detection.ratio
    view = np.zeros(r.shape + (3,), np.uint8)
    view[r > 1.0 + params.brighter_threshold] = STEP_BRIGHT_COLOUR
    view[r < 1.0 - params.darker_threshold] = STEP_DARK_COLOUR
    view[r < 1.0 - params.core_threshold] = STEP_CORE_COLOUR
    return _roi_to_frame(view, detection.roi, shape)


def candidates_view(detection: NutDetection, shape: Tuple[int, int]) -> np.ndarray:
    """Step 3: foreground after morphology, every candidate with its result."""
    mask = None
    if detection.mask is not None:
        mask = cv2.cvtColor((detection.mask > 0).astype(np.uint8) * STEP_MASK_GREY,
                            cv2.COLOR_GRAY2BGR)
    view = _roi_to_frame(mask, detection.roi, shape)
    ambiguous = detection.num_accepted > 1
    for candidate in detection.candidates:
        if candidate.accepted:
            colour = AMBIGUOUS_COLOUR if ambiguous else NUT_COLOUR
            cv2.drawContours(view, [candidate.outer], -1, colour, LINE)
            if candidate.hole is not None:
                cv2.drawContours(view, [candidate.hole], -1, HOLE_COLOUR, 1)
            continue
        cv2.drawContours(view, [candidate.outer], -1, REJECTED_COLOUR, 1)
        if candidate.area_px >= STEP_LABEL_MIN_AREA_PX:
            bx, by, _, _ = cv2.boundingRect(candidate.outer)
            label = f"{candidate.rejected} s/e={candidate.width_ratio:.2f}"
            if candidate.rejected == "size":
                label = (f"size d={candidate.distance_mm:.0f}" if detection.distance_from_size
                         else f"size s={candidate.across_flats_mm:.1f}")
            cv2.putText(view, label, (bx, max(by - 6, 12)), FONT, REJECTION_FONT_SCALE,
                        TEXT_BACKGROUND, 1, cv2.LINE_AA)
    return view


def draw_steps(image_bgr: np.ndarray, detection: NutDetection,
               principal_point: Tuple[float, float], params: NutParams, scale: float,
               fps: Optional[float] = None,
               robot_offset_mm: Optional[Tuple[float, float]] = None) -> np.ndarray:
    """2x2 mosaic of the steps, each tile scaled by scale: I/B | threshold / morphology | result."""
    shape = image_bgr.shape[:2]
    tiles = [ratio_view(detection, shape), threshold_view(detection, params, shape),
             candidates_view(detection, shape),
             draw_debug(image_bgr, detection, principal_point, fps, robot_offset_mm)]
    size = (int(round(shape[1] * scale)), int(round(shape[0] * scale)))
    resized = []
    for tile, title in zip(tiles, STEP_TITLES):
        tile = cv2.resize(tile, size, interpolation=cv2.INTER_AREA)
        _text(tile, [title], (TEXT_MARGIN, size[1] - TEXT_MARGIN), TEXT_COLOUR,
              STEP_TITLE_FONT_SCALE)
        resized.append(tile)
    return np.vstack([np.hstack(resized[:2]), np.hstack(resized[2:])])
