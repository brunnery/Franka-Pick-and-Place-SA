"""Simple object finder: one object on a blank background -> centre of a fitted hexagon.

  ros2 launch software/launch/vision.launch.py

Steps: greyscale -> blur -> background = heavily blurred image (the table) -> band-stop
around the table: foreground where |I - B| / B > threshold (darker or brighter than the
table) -> close/open -> blobs that do not touch the image border (robot parts reaching
into the view, a cut object) and are not specks -> regular hexagon fitted to the convex
hull -> centre of the hexagon.

One object in view: it is taken, no filter. Several objects: only then
  1. shape filter: width ratio of the hull (smallest / largest width over all directions;
     hexagon 0.866, elongated bolt lower, round washer 1.0) in [min_width_ratio,
     max_width_ratio], else rejected (red in the overlay);
  2. one nut-like object left: taken; several: size filter, the one whose width across
     flats (s_px * d / f) is closest to target_across_flats_mm and within size_tolerance
     is taken, the others are nut-like but not chosen (orange).

Hexagon fit: for each orientation theta (0-60 deg), the hull is enclosed by three
pairs of parallel lines with normals theta, theta+60, theta+120. Each pair gives a
width and a midline. The orientation with the smallest total width gives the
tightest enclosing hexagon (it touches the flats of a hexagonal object); its centre
is the least-squares intersection of the three midlines.

Publishes sa_interfaces/Detection on vision/detection (read by sa_tasks) and on
vision/debug either the overlay or
(debug_view "steps") camera image, binary mask and overlay side by side.
"""
import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

try:  # the pure functions above the node run without ROS (pytest on any laptop)
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sa_interfaces.msg import Detection
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:  # pragma: no cover
    Node = object

MM_TO_M = 1e-3
HEXAGON_DIRECTIONS = np.radians([0.0, 60.0, 120.0])
OVERLAY_GREEN, OVERLAY_RED, OVERLAY_YELLOW = (0, 200, 0), (0, 0, 255), (0, 220, 255)
OVERLAY_ORANGE = (0, 140, 255)
# Verdicts of the objects in view (multi-object filter)
CHOSEN, REJECTED_SHAPE, NOT_CHOSEN = "chosen", "shape", "size"


@dataclass
class Hexagon:
    centre: tuple          # (u, v) px
    across_flats_px: float
    angle_deg: float       # normal of the first flat
    regularity: float      # spread of the three widths / their mean (0 = regular hexagon)

    def corners(self) -> np.ndarray:
        radius = self.across_flats_px / math.sqrt(3.0)
        angles = np.radians(self.angle_deg + 30.0 + 60.0 * np.arange(6))
        return np.stack([self.centre[0] + radius * np.cos(angles),
                         self.centre[1] + radius * np.sin(angles)], axis=1)


def fit_hexagon(points: np.ndarray, angle_step_deg: float) -> Hexagon:
    """Tightest regular hexagon around the points (see module docstring)."""
    points = np.asarray(points, float).reshape(-1, 2)
    best = None
    for theta in np.radians(np.arange(0.0, 60.0, angle_step_deg)):
        normals = np.stack([np.cos(theta + HEXAGON_DIRECTIONS),
                            np.sin(theta + HEXAGON_DIRECTIONS)], axis=1)
        projections = points @ normals.T
        low, high = projections.min(axis=0), projections.max(axis=0)
        widths = high - low
        if best is None or widths.sum() < best[0]:
            midlines = (low + high) / 2.0
            centre, *_ = np.linalg.lstsq(normals, midlines, rcond=None)
            best = (widths.sum(), Hexagon(tuple(centre), float(widths.mean()),
                                          math.degrees(theta),
                                          float(widths.std() / widths.mean())))
    return best[1]


@dataclass
class Candidate:
    hull: np.ndarray
    hexagon: Hexagon
    width_ratio: float     # smallest / largest width of the hull
    verdict: str = NOT_CHOSEN


def width_ratio(points: np.ndarray, angle_step_deg: float) -> float:
    """Smallest / largest width of the points over all directions (0-180 deg)."""
    points = np.asarray(points, float).reshape(-1, 2)
    angles = np.radians(np.arange(0.0, 180.0, angle_step_deg))
    projections = points @ np.stack([np.cos(angles), np.sin(angles)])
    widths = projections.max(axis=0) - projections.min(axis=0)
    return float(widths.min() / widths.max())


def choose(candidates: list, p: dict, target_px: Optional[float]) -> tuple:
    """Multi-object filter (module docstring). Sets the verdicts; (chosen or None, status)."""
    nut_like = []
    for candidate in candidates:
        if p["min_width_ratio"] <= candidate.width_ratio <= p["max_width_ratio"]:
            nut_like.append(candidate)
        else:
            candidate.verdict = REJECTED_SHAPE
    n = len(candidates)
    if not nut_like:
        return None, f"{n} objects, none nut-shaped"
    if len(nut_like) == 1:
        nut_like[0].verdict = CHOSEN
        return nut_like[0], f"ok (shape, 1 of {n} objects)"
    if target_px is None:
        return None, f"{len(nut_like)} nut-shaped objects, size unknown"
    errors = [abs(c.hexagon.across_flats_px - target_px) / target_px for c in nut_like]
    best = int(np.argmin(errors))
    if errors[best] > p["size_tolerance"]:
        return None, f"{len(nut_like)} nut-shaped objects, none near the target size"
    nut_like[best].verdict = CHOSEN
    return nut_like[best], f"ok (shape + size, 1 of {n} objects)"


def find_object(image: np.ndarray, p: dict, steps: Optional[dict] = None,
                target_px: Optional[float] = None, objects: Optional[list] = None) -> tuple:
    """(hexagon or None, foreground mask, status). p: the "detection" parameters,
    target_px: target width across flats in px (size filter, only with several objects).
    If steps is a dict, the binary mask before close/open is stored in it (debug view);
    if objects is a list, the Candidates in view are appended to it (overlay)."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (p["blur_kernel_px"],) * 2, 0).astype(np.float32)
    # The background is smooth: blur a downscaled copy (much faster) and scale it back
    scale = p["background_downscale"]
    small = cv2.resize(gray, None, fx=1.0 / scale, fy=1.0 / scale, interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), p["background_sigma_px"] / scale)
    background = cv2.resize(small, gray.shape[::-1], interpolation=cv2.INTER_LINEAR)
    difference = np.abs(gray - background) / np.maximum(background, 1.0)
    mask = (difference > p["threshold"]).astype(np.uint8) * 255
    if steps is not None:
        steps["binary"] = mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p["morph_kernel_px"],) * 2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=p["close_iterations"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = mask.shape
    margin = p["border_margin_px"]

    def inside(contour):
        x, y, w, h = cv2.boundingRect(contour)
        return x > margin and y > margin and x + w < width - margin and y + h < height - margin

    contours = [c for c in contours if inside(c)]
    if not contours:
        return None, mask, "nothing found"
    blobs = [c for c in contours if cv2.contourArea(c) >= p["min_area_px"]]
    if not blobs:
        return None, mask, "nothing found (only specks)"
    candidates = []
    for blob in blobs:
        hull = cv2.convexHull(blob)
        candidates.append(Candidate(hull, fit_hexagon(hull, p["angle_step_deg"]),
                                    width_ratio(hull, p["angle_step_deg"])))
    if objects is not None:
        objects.extend(candidates)
    if len(candidates) == 1:   # one object in view: no filter
        candidates[0].verdict = CHOSEN
        return candidates[0].hexagon, mask, "ok"
    chosen, status = choose(candidates, p, target_px)
    return (chosen.hexagon if chosen else None), mask, status


AXIS_PX = 70        # length of the frame arrows in the overlay
FRAME_MARGIN_PX = 40
FONT, FONT_SCALE = cv2.FONT_HERSHEY_SIMPLEX, 1.0


def _axes(out: np.ndarray, origin: tuple, axes: tuple, colour: tuple) -> None:
    """Two labelled arrows from origin; axes = ((label, (du, dv)), ...) in unit pixels."""
    for label, (du, dv) in axes:
        tip = (int(origin[0] + AXIS_PX * du), int(origin[1] + AXIS_PX * dv))
        cv2.arrowedLine(out, origin, tip, colour, 2, tipLength=0.2)
        # Labels point away from the arrow, and inwards near the right image border
        right = origin[0] > out.shape[1] / 2
        dx = 8 if du > 0 else -45 if du < 0 or right else 8
        cv2.putText(out, label, (tip[0] + dx, tip[1] + (25 if dv > 0 else -8)), FONT, 0.7,
                    colour, 2)


def draw(image: np.ndarray, hexagon: Optional[Hexagon], principal_point: tuple,
         offset_mm: Optional[tuple], distance_mm: float, objects: tuple = (),
         mm_per_px: Optional[float] = None, status: str = "") -> np.ndarray:
    """Minimal overlay: camera frame (top left), robot frame (bottom right), image centre,
    hexagon, and as text only the offset in robot axes and the distance d. With several
    objects: rejected by shape red (label: width ratio), nut-shaped but not chosen orange
    (label: s in mm), chosen green; without a result the status in red."""
    out = image.copy()
    height, width = out.shape[:2]
    m = FRAME_MARGIN_PX
    _axes(out, (m, m), (("x_C", (1, 0)), ("y_C", (0, 1))), OVERLAY_YELLOW)
    # Robot axes in the image: x_robo = -y_cam (up), y_robo = -x_cam (left)
    _axes(out, (width - m, height - m), (("x_R", (0, -1)), ("y_R", (-1, 0))), OVERLAY_GREEN)
    pp = tuple(int(round(c)) for c in principal_point)
    cv2.drawMarker(out, pp, OVERLAY_YELLOW, cv2.MARKER_CROSS, 40, 2)
    lines = []
    if len(objects) > 1:
        for candidate in objects:
            x, y, _, _ = cv2.boundingRect(candidate.hull)
            if candidate.verdict == REJECTED_SHAPE:
                colour, label = OVERLAY_RED, f"w {candidate.width_ratio:.2f}"
            else:
                colour = OVERLAY_GREEN if candidate.verdict == CHOSEN else OVERLAY_ORANGE
                label = (f"{candidate.hexagon.across_flats_px * mm_per_px:.1f} mm"
                         if mm_per_px else f"{candidate.hexagon.across_flats_px:.0f} px")
            if candidate.verdict != CHOSEN:
                cv2.polylines(out, [candidate.hull], True, colour, 2)
            cv2.putText(out, label, (x, max(y - 8, 20)), FONT, 0.7, colour, 2)
    if hexagon is None and status:
        lines.append(status)
    if hexagon is not None:
        centre = tuple(int(round(c)) for c in hexagon.centre)
        cv2.polylines(out, [np.round(hexagon.corners()).astype(np.int32)], True, OVERLAY_GREEN, 2)
        cv2.drawMarker(out, centre, OVERLAY_GREEN, cv2.MARKER_CROSS, 20, 2)
        cv2.line(out, pp, centre, OVERLAY_GREEN, 1)
    if offset_mm is not None:
        lines.append(f"x {offset_mm[0]:+.1f}  y {offset_mm[1]:+.1f} mm")
    lines.append(f"z = {distance_mm:.0f} mm")
    for i, line in enumerate(lines):
        size = cv2.getTextSize(line, FONT, FONT_SCALE, 2)[0]
        cv2.putText(out, line, (width - m - size[0], m + 10 + 40 * i), FONT, FONT_SCALE,
                    OVERLAY_GREEN if offset_mm is not None else OVERLAY_RED, 2)
    return out


def _tile(image: np.ndarray, title: str, scale: float) -> np.ndarray:
    tile = image if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    tile = cv2.resize(tile, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.putText(tile, title, (10, 30), FONT, 0.8, (0, 0, 0), 4)
    cv2.putText(tile, title, (10, 30), FONT, 0.8, OVERLAY_YELLOW, 2)
    return tile


def draw_steps(image: np.ndarray, binary: np.ndarray, result: np.ndarray, threshold: float,
               scale: float) -> np.ndarray:
    """One row: camera image | binary mask | result with the hexagon."""
    return np.hstack([_tile(image, "camera", scale),
                      _tile(binary, f"table 1 +- {threshold:.2f} removed", scale),
                      _tile(result, "result", scale)])


DETECTION_PARAMETERS = ("blur_kernel_px", "background_sigma_px", "background_downscale", "threshold",
                        "morph_kernel_px", "close_iterations", "min_area_px", "border_margin_px",
                        "angle_step_deg", "min_width_ratio", "max_width_ratio",
                        "target_across_flats_mm", "size_tolerance")
DOUBLE_PARAMETERS = ("background_sigma_px", "threshold", "angle_step_deg", "min_width_ratio",
                     "max_width_ratio", "target_across_flats_mm", "size_tolerance")


class SimpleVisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        for name in DETECTION_PARAMETERS:
            kind = Parameter.Type.DOUBLE if name in DOUBLE_PARAMETERS else Parameter.Type.INTEGER
            self.declare_parameter(f"detection.{name}", kind)
        self.declare_parameter("publish_debug", Parameter.Type.BOOL)
        self.declare_parameter("debug_view", Parameter.Type.STRING)
        self.declare_parameter("debug_steps_scale", Parameter.Type.DOUBLE)
        # Set by sa_tasks; used for point_camera and the size filter (several objects)
        self.declare_parameter("object_plane_distance_mm", Parameter.Type.DOUBLE)
        self.declare_parameter("target", Parameter.Type.STRING)
        self.declare_parameter("d_source", Parameter.Type.STRING)
        self.bridge = CvBridge()
        self.k = self.dist = None
        self.pub = self.create_publisher(Detection, "vision/detection", 10)
        self.debug_pub = self.create_publisher(Image, "vision/debug", 1)
        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        self.k = np.asarray(msg.k, float).reshape(3, 3)
        self.dist = np.asarray(msg.d, float)

    def on_image(self, msg):
        if self.k is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        p = {name: self.get_parameter(f"detection.{name}").value for name in DETECTION_PARAMETERS}
        steps = {} if self.get_parameter("debug_view").value == "steps" else None
        d_mm = self.get_parameter("object_plane_distance_mm").value
        mm_per_px = d_mm / self.k[0, 0]
        objects = []
        hexagon, mask, status = find_object(image, p, steps,
                                            p["target_across_flats_mm"] / mm_per_px, objects)
        out = Detection(header=msg.header, valid=hexagon is not None, status=status)
        nan = float("nan")
        out.u = out.v = out.u_undistorted = out.v_undistorted = nan
        out.point_camera.x = out.point_camera.y = out.point_camera.z = nan
        out.across_flats_mm = out.area_px = out.hole_area_px = nan
        out.width_ratio = out.compactness = nan
        out.plane_distance = d_mm * MM_TO_M
        out.num_candidates = sum(c.verdict != REJECTED_SHAPE for c in objects)
        offset = None
        if hexagon is not None:
            out.u, out.v = hexagon.centre
            und = cv2.undistortPoints(np.array([[hexagon.centre]], float), self.k, self.dist,
                                      P=self.k)[0, 0]
            out.u_undistorted, out.v_undistorted = float(und[0]), float(und[1])
            ray = np.linalg.solve(self.k, [und[0], und[1], 1.0]) * d_mm * MM_TO_M
            out.point_camera.x, out.point_camera.y, out.point_camera.z = ray.tolist()
            out.across_flats_mm = hexagon.across_flats_px * mm_per_px
            # Offset of the object from the image centre in robot axes:
            # x_robo = -y_cam, y_robo = -x_cam (CLAUDE.md)
            offset = (-ray[1] / MM_TO_M, -ray[0] / MM_TO_M)
            out.width_ratio = hexagon.regularity
        self.pub.publish(out)
        if self.get_parameter("publish_debug").value:
            debug = draw(image, hexagon, (self.k[0, 2], self.k[1, 2]), offset, d_mm, objects,
                         mm_per_px, status)
            if steps is not None:
                debug = draw_steps(image, steps["binary"], debug, p["threshold"],
                                   self.get_parameter("debug_steps_scale").value)
            debug = self.bridge.cv2_to_imgmsg(debug, "bgr8")
            debug.header = msg.header
            self.debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(SimpleVisionNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
