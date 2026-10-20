"""Simple object finder: one object on a blank background -> centre of a fitted hexagon
(target nut) or square (target stud).

  ros2 launch software/launch/vision.launch.py

Steps: greyscale -> blur -> background = heavily blurred image (the table) -> band-stop
around the table: foreground where |I - B| / B > threshold (darker or brighter than the
table) -> close/open -> blobs that do not touch the image border (robot parts reaching
into the view, a cut object) and are not specks -> regular hexagon fitted to the convex
hull (square for the stud) -> its centre.

One object in view: it is taken, no filter. Several objects: only then
  1. shape filter: width ratio of the hull (smallest / largest width over all directions;
     hexagon 0.866, elongated bolt lower, round washer 1.0) in [min_width_ratio,
     max_width_ratio], else rejected (red in the overlay);
  2. one target-like object left: taken; several: size filter, the one whose width across
     flats (s_px * d / f) is closest to target_size_mm and within size_tolerance is
     taken, the others are target-like but not chosen (orange).
Shape band and size are per target (vision.yaml: targets.nut, targets.stud).
Objects are tracked over the images (Tracker): only objects found in the last
tracking.frames images count, with the median of those positions and sizes. After
tracking.lock_after_frames images the result is locked (fixed) until the next request. In addition,
every image lists the objects that pass a loose filter (no specks, no lines)
in Detection.candidates, ranked by area, largest first (numbered in the overlay): sa_tasks
centres the camera over them one by one for the stud, whose outline is only a clean square
when seen from directly above.

Hexagon fit: for each orientation theta (0-60 deg), the hull is enclosed by three
pairs of parallel lines with normals theta, theta+60, theta+120. Each pair gives a
width and a midline. The orientation with the smallest total width gives the
tightest enclosing hexagon (it touches the flats of a hexagonal object); its centre
is the least-squares intersection of the three midlines. The stud (square base) is
fitted the same way with two pairs (theta, theta+90): the tightest enclosing square.
The parameter "target" (nut | stud, set by sa_tasks) selects the polygon and the
shape and size of the multi-object filter.

Publishes sa_interfaces/Detection on vision/detection (read by sa_tasks) and on
vision/debug either the overlay or
(debug_view "steps") camera image and binary mask side by side, the overlay large below.
"""
import math
from collections import deque
from dataclasses import dataclass, replace
from typing import Optional

import cv2
import numpy as np

try:  # the pure functions above the node run without ROS (pytest on any laptop)
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.qos import qos_profile_sensor_data
    from sa_interfaces.msg import Candidate as CandidateMsg, Detection
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:  # pragma: no cover
    Node = object

MM_TO_M = 1e-3
OVERLAY_GREEN, OVERLAY_RED, OVERLAY_YELLOW = (0, 200, 0), (0, 0, 255), (0, 220, 255)
OVERLAY_ORANGE = (0, 140, 255)
# Verdicts of the objects in view (multi-object filter)
CHOSEN, REJECTED_SHAPE, NOT_CHOSEN = "chosen", "shape", "size"


@dataclass
class Polygon:
    """Regular polygon with an even number of sides (hexagon: nut, square: stud base)."""
    centre: tuple          # (u, v) px
    across_flats_px: float # width across flats (square: side length)
    angle_deg: float       # normal of the first flat
    regularity: float      # spread of the widths / their mean (0 = regular polygon)
    sides: int = 6

    def corners(self) -> np.ndarray:
        half_angle = math.pi / self.sides
        radius = self.across_flats_px / 2.0 / math.cos(half_angle)
        angles = np.radians(self.angle_deg + math.degrees(half_angle)
                            + 360.0 / self.sides * np.arange(self.sides))
        return np.stack([self.centre[0] + radius * np.cos(angles),
                         self.centre[1] + radius * np.sin(angles)], axis=1)


def fit_polygon(points: np.ndarray, sides: int, angle_step_deg: float) -> Polygon:
    """Tightest regular polygon with an even number of sides around the points: sides / 2
    pairs of parallel lines, 180 / (sides / 2) deg apart (see module docstring)."""
    points = np.asarray(points, float).reshape(-1, 2)
    pairs = sides // 2
    directions = np.radians(180.0 / pairs * np.arange(pairs))
    best = None
    for theta in np.radians(np.arange(0.0, 180.0 / pairs, angle_step_deg)):
        normals = np.stack([np.cos(theta + directions), np.sin(theta + directions)], axis=1)
        projections = points @ normals.T
        low, high = projections.min(axis=0), projections.max(axis=0)
        widths = high - low
        if best is None or widths.sum() < best[0]:
            midlines = (low + high) / 2.0
            centre, *_ = np.linalg.lstsq(normals, midlines, rcond=None)
            best = (widths.sum(), Polygon(tuple(centre), float(widths.mean()),
                                          math.degrees(theta),
                                          float(widths.std() / widths.mean()), sides))
    return best[1]


def fit_hexagon(points: np.ndarray, angle_step_deg: float) -> Polygon:
    return fit_polygon(points, 6, angle_step_deg)


@dataclass
class Candidate:
    hull: np.ndarray
    polygon: Polygon
    width_ratio: float     # smallest / largest width of the hull
    verdict: str = NOT_CHOSEN
    shape_ok: bool = False # shape and size within the target's band (set by rank)
    rank: int = 0          # place in Detection.candidates, 1 = most likely; 0 = none
    seen: int = 1          # images of its track in the median (Tracker)


def width_ratio(points: np.ndarray, angle_step_deg: float) -> float:
    """Smallest / largest width of the points over all directions (0-180 deg)."""
    points = np.asarray(points, float).reshape(-1, 2)
    angles = np.radians(np.arange(0.0, 180.0, angle_step_deg))
    projections = points @ np.stack([np.cos(angles), np.sin(angles)])
    widths = projections.max(axis=0) - projections.min(axis=0)
    return float(widths.min() / widths.max())


class Track:
    """One object followed from image to image."""

    def __init__(self, candidate, frames: int):
        self.samples = deque(maxlen=frames)  # (centre, across_flats_px, width_ratio)
        self.latest = candidate              # newest detection (hull, polygon, verdict)
        self.misses = 0                      # images in a row without a match
        self.add(candidate)

    def add(self, candidate) -> None:
        self.latest = candidate
        self.misses = 0
        self.samples.append((candidate.polygon.centre, candidate.polygon.across_flats_px,
                             candidate.width_ratio))


class Tracker:
    """Objects followed over the images: each object of a new image is matched to the track
    whose last centre is nearest (within match_radius_px, nearest pairs first). A track's
    centre, width across flats and width ratio are the medians over the last `frames`
    images in which it was found; it is confirmed once it has `frames` such images. A track
    not found for more than max_misses images in a row is dropped. Specks that flicker never
    get confirmed (20.10.: the stud search locked on specks in 2 of 3 runs). reset() starts
    over (each request of sa_tasks, after the robot has settled)."""

    def __init__(self, frames: int, match_radius_px: float, max_misses: int):
        self.frames, self.match_radius_px, self.max_misses = frames, match_radius_px, max_misses
        self.reset()

    def reset(self) -> None:
        self.tracks = []
        self.frames_since_reset = 0

    def update(self, candidates: list) -> list:
        """Adds an image; returns one Candidate per live track (copy of its newest
        detection with the median centre, size and width ratio, `seen` = its samples)."""
        self.frames_since_reset += 1
        pairs = sorted((math.dist(t.samples[-1][0], c.polygon.centre), ti, ci)
                       for ti, t in enumerate(self.tracks) for ci, c in enumerate(candidates))
        matched_tracks, matched = set(), set()
        for distance, ti, ci in pairs:
            if distance > self.match_radius_px:
                break
            if ti in matched_tracks or ci in matched:
                continue
            self.tracks[ti].add(candidates[ci])
            matched_tracks.add(ti)
            matched.add(ci)
        for ti, track in enumerate(self.tracks):
            if ti not in matched_tracks:
                track.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]
        self.tracks += [Track(c, self.frames) for ci, c in enumerate(candidates)
                        if ci not in matched]
        out = []
        for track in self.tracks:
            samples = list(track.samples)
            centre = tuple(np.median([s[0] for s in samples], axis=0))
            polygon = replace(track.latest.polygon, centre=centre,
                              across_flats_px=float(np.median([s[1] for s in samples])))
            out.append(replace(track.latest, polygon=polygon,
                               width_ratio=float(np.median([s[2] for s in samples])),
                               seen=len(samples),
                               # a verdict from an older image does not count now
                               verdict=track.latest.verdict if track.misses == 0
                               else NOT_CHOSEN))
        return out


def rank(candidates: list, p: dict, target_px: float, mm_per_px: float) -> list:
    """Candidates for the search, most likely target first. A loose filter first drops specks
    and lines (search_min/max_size_mm, search_min_width_ratio: the tall stud seen off-axis
    is no clean square, so the strict band cannot be used here). The rest is sorted by the
    area of the hull, largest first: the stud is by far the largest dark object, and the
    order stays the same from frame to frame (sorting square-shaped first let a speck that
    happened to look square jump to #1, 20.10.). Sets shape_ok (strict band, judged by
    sa_tasks only with the camera centred) and rank (1, 2, ...; 0 = no candidate)."""
    def size_error(candidate):
        return abs(candidate.polygon.across_flats_px - target_px) / target_px

    ranked = []
    for candidate in candidates:
        size_mm = candidate.polygon.across_flats_px * mm_per_px
        # bool(): numpy bools are refused by the ROS message
        candidate.shape_ok = bool(p["min_width_ratio"] <= candidate.width_ratio
                                  <= p["max_width_ratio"]
                                  and size_error(candidate) <= p["size_tolerance"])
        candidate.rank = 0
        if (p["search_min_size_mm"] <= size_mm <= p["search_max_size_mm"]
                and candidate.width_ratio >= p["search_min_width_ratio"]):
            ranked.append(candidate)
    ranked.sort(key=lambda c: -cv2.contourArea(c.hull))
    for number, candidate in enumerate(ranked, 1):
        candidate.rank = number
    return ranked


def choose(candidates: list, p: dict, target_px: Optional[float], target: str = "nut") -> tuple:
    """Multi-object filter (module docstring). Sets the verdicts; (chosen or None, status)."""
    shaped = []
    for candidate in candidates:
        if p["min_width_ratio"] <= candidate.width_ratio <= p["max_width_ratio"]:
            shaped.append(candidate)
        else:
            candidate.verdict = REJECTED_SHAPE
    n = len(candidates)
    if not shaped:
        return None, f"{n} objects, none {target}-shaped"
    if len(shaped) == 1:
        shaped[0].verdict = CHOSEN
        return shaped[0], f"ok (shape, 1 of {n} objects)"
    if target_px is None:
        return None, f"{len(shaped)} {target}-shaped objects, size unknown"
    errors = [abs(c.polygon.across_flats_px - target_px) / target_px for c in shaped]
    best = int(np.argmin(errors))
    if errors[best] > p["size_tolerance"]:
        return None, f"{len(shaped)} {target}-shaped objects, none near the target size"
    shaped[best].verdict = CHOSEN
    return shaped[best], f"ok (shape + size, 1 of {n} objects)"


def find_object(image: np.ndarray, p: dict, steps: Optional[dict] = None,
                target_px: Optional[float] = None, objects: Optional[list] = None,
                target: str = "nut") -> tuple:
    """(polygon or None, foreground mask, status). p: the "detection" parameters merged
    with those of the target (sides, shape band, size tolerance), target_px: target width
    across flats in px (size filter, only with several objects).
    If steps is a dict, the binary mask before close/open is stored in it (debug view);
    if objects is a list, the Candidates in view are appended to it (overlay)."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (p["blur_kernel_px"],) * 2, 0).astype(np.float32)
    # The background is smooth: blur a downscaled copy (much faster) and scale it back
    scale = p["background_downscale"]
    small = cv2.resize(gray, None, fx=1.0 / scale, fy=1.0 / scale, interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), p["background_sigma_px"] / scale)
    background = cv2.resize(small, gray.shape[::-1], interpolation=cv2.INTER_LINEAR)
    ratio = gray / np.maximum(background, 1.0)
    if p["segmentation"] == "dark":  # only darker than dark_threshold: drops soft shadows
        mask = (ratio < p["dark_threshold"]).astype(np.uint8) * 255
    else:                            # "band": darker or brighter than the table
        mask = (np.abs(ratio - 1.0) > p["threshold"]).astype(np.uint8) * 255
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
        candidates.append(Candidate(hull, fit_polygon(hull, p["sides"], p["angle_step_deg"]),
                                    width_ratio(hull, p["angle_step_deg"])))
    if objects is not None:
        objects.extend(candidates)
    if len(candidates) == 1:   # one object in view: no filter
        candidates[0].verdict = CHOSEN
        return candidates[0].polygon, mask, "ok"
    chosen, status = choose(candidates, p, target_px, target)
    return (chosen.polygon if chosen else None), mask, status


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


def draw(image: np.ndarray, polygon: Optional[Polygon], principal_point: tuple,
         offset_mm: Optional[tuple], distance_mm: float, objects: tuple = (),
         mm_per_px: Optional[float] = None, status: str = "",
         inner_circle_mm: float = 0.0) -> np.ndarray:
    """Minimal overlay: camera frame (top left), robot frame (bottom right), image centre,
    fitted polygon, and as text only the offset in robot axes and the distance d. With several
    objects: rejected by shape red (label: width ratio), target-shaped but not chosen orange
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
    if objects:  # labels also for a single object (size, width ratio)
        for candidate in objects:
            x, y, _, _ = cv2.boundingRect(candidate.hull)
            if candidate.verdict == REJECTED_SHAPE:
                colour, label = OVERLAY_RED, f"w {candidate.width_ratio:.2f}"
            else:
                colour = OVERLAY_GREEN if candidate.verdict == CHOSEN else OVERLAY_ORANGE
                label = (f"{candidate.polygon.across_flats_px * mm_per_px:.1f} mm"
                         if mm_per_px else f"{candidate.polygon.across_flats_px:.0f} px")
                label += f" w {candidate.width_ratio:.2f}"
            if candidate.verdict != CHOSEN:
                cv2.polylines(out, [candidate.hull], True, colour, 2)
            # Number = rank in Detection.candidates (stud search order)
            if candidate.rank:
                label = f"#{candidate.rank} {label}"
            cv2.putText(out, label, (x, max(y - 8, 20)), FONT, 0.7, colour, 2)
    if polygon is None and status:
        lines.append(status)
    if polygon is not None:
        centre = tuple(int(round(c)) for c in polygon.centre)
        cv2.polylines(out, [np.round(polygon.corners()).astype(np.int32)], True, OVERLAY_GREEN, 2)
        if inner_circle_mm > 0 and mm_per_px:  # stud: the thread inside the square base
            cv2.circle(out, centre, int(round(inner_circle_mm / 2.0 / mm_per_px)), OVERLAY_GREEN, 2)
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


def segmentation_caption(p: dict) -> str:
    if p["segmentation"] == "dark":
        return f"darker than {p['dark_threshold']:.2f} x table"
    return f"table 1 +- {p['threshold']:.2f} removed"


def draw_steps(image: np.ndarray, binary: np.ndarray, result: np.ndarray, caption: str,
               scale: float) -> np.ndarray:
    """Top row: camera image | binary mask; below: the result with the polygon at twice the
    scale, as wide as the top row."""
    top = np.hstack([_tile(image, "camera", scale),
                     _tile(binary, caption, scale)])
    bottom = _tile(result, "result", 2.0 * scale)
    # Rounding of cv2.resize can differ by a pixel between the two rows
    if bottom.shape[1] != top.shape[1]:
        height = round(bottom.shape[0] * top.shape[1] / bottom.shape[1])
        bottom = cv2.resize(bottom, (top.shape[1], height))
    return np.vstack([top, bottom])


DETECTION_PARAMETERS = ("blur_kernel_px", "background_sigma_px", "background_downscale", "threshold",
                        "morph_kernel_px", "close_iterations", "min_area_px", "border_margin_px",
                        "angle_step_deg")
# Per target (vision.yaml: targets.<nut|stud>.<name>)
TARGET_PARAMETERS = ("sides", "segmentation", "dark_threshold", "inner_circle_mm",
                     "min_width_ratio", "max_width_ratio", "target_size_mm",
                     "size_tolerance", "search_min_size_mm", "search_max_size_mm",
                     "search_min_width_ratio")
TARGETS = ("nut", "stud")
DOUBLE_PARAMETERS = ("background_sigma_px", "threshold", "angle_step_deg", "min_width_ratio",
                     "max_width_ratio", "target_size_mm", "size_tolerance", "search_min_size_mm",
                     "search_max_size_mm", "search_min_width_ratio", "dark_threshold",
                     "inner_circle_mm")
STRING_PARAMETERS = ("segmentation",)


def parameter_kind(name: str):
    if name in STRING_PARAMETERS:
        return Parameter.Type.STRING
    return Parameter.Type.DOUBLE if name in DOUBLE_PARAMETERS else Parameter.Type.INTEGER


class SimpleVisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        for name in DETECTION_PARAMETERS:
            self.declare_parameter(f"detection.{name}", parameter_kind(name))
        for target in TARGETS:
            for name in TARGET_PARAMETERS:
                self.declare_parameter(f"targets.{target}.{name}", parameter_kind(name))
        self.declare_parameter("publish_debug", Parameter.Type.BOOL)
        self.declare_parameter("debug_view", Parameter.Type.STRING)
        self.declare_parameter("debug_steps_scale", Parameter.Type.DOUBLE)
        self.declare_parameter("tracking.frames", Parameter.Type.INTEGER)
        self.declare_parameter("tracking.match_radius_px", Parameter.Type.DOUBLE)
        self.declare_parameter("tracking.max_misses", Parameter.Type.INTEGER)
        self.declare_parameter("tracking.lock_after_frames", Parameter.Type.INTEGER)
        # Set by sa_tasks; used for point_camera, the size filter (several objects) and the
        # choice of polygon and filters (target)
        self.declare_parameter("object_plane_distance_mm", Parameter.Type.DOUBLE)
        self.declare_parameter("target", Parameter.Type.STRING)
        self.declare_parameter("lock", Parameter.Type.BOOL)
        self.declare_parameter("d_source", Parameter.Type.STRING)
        self.bridge = CvBridge()
        self.k = self.dist = None
        self.tracker = Tracker(self.get_parameter("tracking.frames").value,
                               self.get_parameter("tracking.match_radius_px").value,
                               self.get_parameter("tracking.max_misses").value)
        self.reset_tracks = True
        self.locked = None  # (polygon, objects, ranked, status) fixed until the next reset
        # sa_tasks sets target and distance with every request: start the tracks over then
        self.add_on_set_parameters_callback(self.on_parameters)
        self.pub = self.create_publisher(Detection, "vision/detection", 10)
        self.debug_pub = self.create_publisher(Image, "vision/debug", 1)
        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)

    def on_parameters(self, parameters):
        if any(p.name in ("target", "object_plane_distance_mm", "lock") for p in parameters):
            self.reset_tracks = True
        return SetParametersResult(successful=True)

    def on_info(self, msg):
        self.k = np.asarray(msg.k, float).reshape(3, 3)
        self.dist = np.asarray(msg.d, float)

    def on_image(self, msg):
        if self.k is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        target = self.get_parameter("target").value
        if target not in TARGETS:
            self.get_logger().error(f"target must be one of {TARGETS}, not '{target}'",
                                    throttle_duration_sec=5.0)
            return
        p = {name: self.get_parameter(f"detection.{name}").value for name in DETECTION_PARAMETERS}
        p.update({name: self.get_parameter(f"targets.{target}.{name}").value
                  for name in TARGET_PARAMETERS})
        steps = {} if self.get_parameter("debug_view").value == "steps" else None
        d_mm = self.get_parameter("object_plane_distance_mm").value
        mm_per_px = d_mm / self.k[0, 0]
        objects = []
        target_px = p["target_size_mm"] / mm_per_px
        polygon, mask, status = find_object(image, p, steps, target_px, objects, target)
        if self.reset_tracks:
            self.tracker.reset()
            self.reset_tracks = False
            self.locked = None
        # lock (choosing among objects): only confirmed tracks count, and the result is locked
        # below. Else (measuring a centred object): every track, with the median over its
        # last images
        lock = self.get_parameter("lock").value
        frames = self.tracker.frames
        objects = [c for c in self.tracker.update(objects) if c.seen >= frames or not lock]
        chosen = next((c for c in objects if c.verdict == CHOSEN), None)
        if polygon is not None:
            polygon = chosen.polygon if chosen else None
            if chosen is None:
                status = f"confirming ({self.tracker.frames_since_reset} images)"
        ranked = rank(objects, p, target_px, mm_per_px)
        if target == "stud":
            # The stud is searched by centring (sa_tasks) over the ranked candidates; the
            # detection shown and published is the top-ranked one
            polygon = ranked[0].polygon if ranked else None
            status = f"{len(ranked)} candidate(s)" if ranked else "no candidate"
            for candidate in objects:
                candidate.verdict = (CHOSEN if candidate.rank == 1 else NOT_CHOSEN
                                     if candidate.rank else REJECTED_SHAPE)
        # Lock: once lock_after_frames images have been tracked and the target is confirmed,
        # its medians are fixed until the next request of sa_tasks (only locked detections
        # are used there). Before that the confirmed tracks are shown, not published as valid.
        if (lock and self.locked is None and polygon is not None and self.tracker.frames_since_reset
                >= self.get_parameter("tracking.lock_after_frames").value):
            self.locked = (polygon, objects, ranked, status)
        if self.locked is not None:
            polygon, objects, ranked, status = self.locked
            status = f"LOCKED, {status}"
        elif lock:
            status = f"tracking ({self.tracker.frames_since_reset} images), {status}"
        out = Detection(header=msg.header, valid=polygon is not None, status=status)
        out.locked = self.locked is not None
        nan = float("nan")
        out.u = out.v = out.u_undistorted = out.v_undistorted = nan
        out.point_camera.x = out.point_camera.y = out.point_camera.z = nan
        out.across_flats_mm = out.area_px = out.hole_area_px = nan
        out.width_ratio = out.compactness = nan
        out.plane_distance = d_mm * MM_TO_M
        out.num_candidates = sum(c.verdict != REJECTED_SHAPE for c in objects)
        out.frames_since_reset = self.tracker.frames_since_reset
        out.frames_window = frames
        if ranked:
            centres = np.array([c.polygon.centre for c in ranked], float).reshape(-1, 1, 2)
            undistorted = cv2.undistortPoints(centres, self.k, self.dist, P=self.k)[:, 0]
            for candidate, (u, v) in zip(ranked, undistorted):
                out.candidates.append(CandidateMsg(
                    u_undistorted=float(u), v_undistorted=float(v),
                    size_mm=float(candidate.polygon.across_flats_px * mm_per_px),
                    width_ratio=float(candidate.width_ratio), shape_ok=candidate.shape_ok))
        offset = None
        if polygon is not None:
            out.u, out.v = polygon.centre
            und = cv2.undistortPoints(np.array([[polygon.centre]], float), self.k, self.dist,
                                      P=self.k)[0, 0]
            out.u_undistorted, out.v_undistorted = float(und[0]), float(und[1])
            ray = np.linalg.solve(self.k, [und[0], und[1], 1.0]) * d_mm * MM_TO_M
            out.point_camera.x, out.point_camera.y, out.point_camera.z = ray.tolist()
            out.across_flats_mm = polygon.across_flats_px * mm_per_px
            # Offset of the object from the image centre in robot axes:
            # x_robo = -y_cam, y_robo = -x_cam (CLAUDE.md)
            offset = (-ray[1] / MM_TO_M, -ray[0] / MM_TO_M)
            out.width_ratio = polygon.regularity
        self.pub.publish(out)
        if self.get_parameter("publish_debug").value:
            debug = draw(image, polygon, (self.k[0, 2], self.k[1, 2]), offset, d_mm, objects,
                         mm_per_px, f"{target}: {status}", p["inner_circle_mm"])
            if lock:
                label = "LOCKED" if self.locked is not None else "tracking"
                colour = OVERLAY_GREEN if self.locked is not None else OVERLAY_YELLOW
                cv2.putText(debug, label, (20, debug.shape[0] - 20), FONT, 1.5, (0, 0, 0), 8)
                cv2.putText(debug, label, (20, debug.shape[0] - 20), FONT, 1.5, colour, 3)
            if steps is not None:
                debug = draw_steps(image, steps["binary"], debug, segmentation_caption(p),
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
