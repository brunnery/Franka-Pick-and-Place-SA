"""Object finder on a blank table: centre of a fitted hexagon (target nut) or square (target
stud). docs/THEORY.md Section 2.0.

  ros2 launch software/launch/vision.launch.py

Steps: greyscale -> blur -> background = heavily blurred image (the table) -> foreground
where |I/B - 1| > threshold (nut: darker or brighter than the table) or I/B < dark_threshold
(stud: dark only, drops its soft shadow) -> close/open -> blobs off the image border and
larger than specks -> regular polygon fitted to the convex hull -> its centre.

Polygon fit: for each orientation theta, the hull is enclosed by sides/2 pairs of parallel
lines 180/(sides/2) deg apart; the orientation with the smallest total width gives the
tightest enclosing polygon, its centre the least-squares intersection of the midlines.

Nut: one object in view is taken; with several, only those whose width ratio (smallest /
largest hull width) lies in the target's band count, and of several of those the one
closest to target_size_mm. Stud: all objects that pass a loose filter (no specks, no lines)
are ranked by area and published in Detection.candidates; sa_tasks centres the camera over
them one by one, because the tall stud is only a clean square from directly above.

Objects are tracked over the images (Tracker, median of the last tracking.frames finds).
sa_tasks sets target, distance and lock with every request, which starts the tracks over.
lock: only tracks found tracking.frames times count, and after lock_after_frames images the
result is fixed until the next request.
"""
import math
from collections import deque
from dataclasses import dataclass, replace
from typing import Optional

import cv2
import numpy as np

try:  # the pure functions run without ROS (pytest on any laptop)
    import rclpy
    from cv_bridge import CvBridge
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sa_interfaces.msg import Candidate as CandidateMsg
    from sa_interfaces.msg import Detection
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:  # pragma: no cover
    Node = object

GREEN, RED, YELLOW, ORANGE = (0, 200, 0), (0, 0, 255), (0, 220, 255), (0, 140, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX
CHOSEN, REJECTED, NOT_CHOSEN = "chosen", "rejected", "not chosen"


@dataclass
class Polygon:
    """Regular polygon with an even number of sides (hexagon: nut, square: stud base)."""
    centre: tuple           # (u, v) px
    across_flats_px: float  # square: side length
    angle_deg: float        # normal of the first flat
    sides: int

    def corners(self) -> np.ndarray:
        half = math.pi / self.sides
        radius = self.across_flats_px / 2.0 / math.cos(half)
        angles = np.radians(self.angle_deg) + half + 2 * half * np.arange(self.sides)
        return np.stack([self.centre[0] + radius * np.cos(angles),
                         self.centre[1] + radius * np.sin(angles)], axis=1)


def fit_polygon(points: np.ndarray, sides: int, angle_step_deg: float) -> Polygon:
    """Tightest regular polygon with an even number of sides around the points."""
    points = np.asarray(points, float).reshape(-1, 2)
    pairs = sides // 2
    directions = np.radians(180.0 / pairs * np.arange(pairs))
    best = None
    for theta in np.radians(np.arange(0.0, 180.0 / pairs, angle_step_deg)):
        normals = np.stack([np.cos(theta + directions), np.sin(theta + directions)], axis=1)
        projections = points @ normals.T
        low, high = projections.min(axis=0), projections.max(axis=0)
        if best is None or (high - low).sum() < best[0]:
            centre = np.linalg.lstsq(normals, (low + high) / 2.0, rcond=None)[0]
            best = ((high - low).sum(), Polygon(tuple(centre), float((high - low).mean()),
                                                math.degrees(theta), sides))
    return best[1]


def width_ratio(points: np.ndarray, angle_step_deg: float) -> float:
    """Smallest / largest width of the points over all directions (0-180 deg)."""
    angles = np.radians(np.arange(0.0, 180.0, angle_step_deg))
    projections = np.asarray(points, float).reshape(-1, 2) @ np.stack([np.cos(angles),
                                                                        np.sin(angles)])
    widths = projections.max(axis=0) - projections.min(axis=0)
    return float(widths.min() / widths.max())


@dataclass
class Candidate:
    hull: np.ndarray
    polygon: Polygon
    width_ratio: float
    verdict: str = NOT_CHOSEN
    shape_ok: bool = False  # strict shape and size band of the target (stud, set by rank)
    rank: int = 0           # place in Detection.candidates, 1 = most likely; 0 = none
    seen: int = 1           # finds of its track in the median (Tracker)


class Track:
    """One object followed from image to image."""

    def __init__(self, candidate: Candidate, frames: int):
        self.samples = deque(maxlen=frames)  # (centre, across_flats_px, width_ratio)
        self.add(candidate)

    def add(self, candidate: Candidate) -> None:
        self.latest, self.misses = candidate, 0
        self.samples.append((candidate.polygon.centre, candidate.polygon.across_flats_px,
                             candidate.width_ratio))


class Tracker:
    """Each object of a new image is matched to the track whose last centre is nearest
    (within match_radius_px, nearest pairs first). A track's centre, size and width ratio
    are the medians over its last `frames` finds; a track missing more than max_misses
    images in a row ends. Flickering specks never reach `frames` finds."""

    def __init__(self, frames: int, match_radius_px: float, max_misses: int):
        self.frames, self.match_radius_px, self.max_misses = frames, match_radius_px, max_misses
        self.reset()

    def reset(self) -> None:
        self.tracks, self.frames_since_reset = [], 0

    def update(self, candidates: list) -> list:
        """Adds an image; returns one Candidate per live track (its newest detection with the
        medians, `seen` = its finds)."""
        self.frames_since_reset += 1
        pairs = sorted((math.dist(t.samples[-1][0], c.polygon.centre), ti, ci)
                       for ti, t in enumerate(self.tracks) for ci, c in enumerate(candidates))
        matched_tracks, matched = set(), set()
        for distance, ti, ci in pairs:
            if distance <= self.match_radius_px and ti not in matched_tracks and ci not in matched:
                self.tracks[ti].add(candidates[ci])
                matched_tracks.add(ti)
                matched.add(ci)
        for ti, track in enumerate(self.tracks):
            track.misses += ti not in matched_tracks
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]
        self.tracks += [Track(c, self.frames) for ci, c in enumerate(candidates)
                        if ci not in matched]
        out = []
        for track in self.tracks:
            centres, sizes, ratios = zip(*track.samples)
            polygon = replace(track.latest.polygon, centre=tuple(np.median(centres, axis=0)),
                              across_flats_px=float(np.median(sizes)))
            out.append(replace(track.latest, polygon=polygon, width_ratio=float(np.median(ratios)),
                               seen=len(track.samples),
                               verdict=track.latest.verdict if track.misses == 0 else NOT_CHOSEN))
        return out


def in_band(candidate: Candidate, p: dict) -> bool:
    return p["min_width_ratio"] <= candidate.width_ratio <= p["max_width_ratio"]


def size_error(candidate: Candidate, target_px: float) -> float:
    return abs(candidate.polygon.across_flats_px - target_px) / target_px


def rank(candidates: list, p: dict, target_px: float, mm_per_px: float) -> list:
    """Stud: candidates that pass the loose filter (search_min/max_size_mm,
    search_min_width_ratio), largest hull first (stable from image to image). Sets rank,
    the verdicts and shape_ok (strict band, judged by sa_tasks with the camera centred)."""
    ranked = []
    for c in candidates:
        c.shape_ok = bool(in_band(c, p) and size_error(c, target_px) <= p["size_tolerance"])
        c.rank, c.verdict = 0, REJECTED
        if (p["search_min_size_mm"] <= c.polygon.across_flats_px * mm_per_px
                <= p["search_max_size_mm"] and c.width_ratio >= p["search_min_width_ratio"]):
            ranked.append(c)
    ranked.sort(key=lambda c: -cv2.contourArea(c.hull))
    for number, c in enumerate(ranked, 1):
        c.rank, c.verdict = number, CHOSEN if number == 1 else NOT_CHOSEN
    return ranked


def choose(candidates: list, p: dict, target_px: float) -> str:
    """Nut: one object is taken; of several, those in the shape band, and of several of
    those the one closest to the target size. Sets the verdicts; returns the status."""
    if len(candidates) == 1:
        candidates[0].verdict = CHOSEN
        return "ok"
    shaped = [c for c in candidates if in_band(c, p)]
    for c in candidates:
        c.verdict = NOT_CHOSEN if in_band(c, p) else REJECTED
    if not shaped:
        return f"{len(candidates)} objects, none nut-shaped"
    best = min(shaped, key=lambda c: size_error(c, target_px))
    if size_error(best, target_px) > p["size_tolerance"]:
        return f"{len(shaped)} nut-shaped objects, none near the target size"
    best.verdict = CHOSEN
    return f"ok ({len(candidates)} objects)"


def find_objects(image: np.ndarray, p: dict) -> tuple:
    """(binary mask before close/open, Candidates). p: the "detection" parameters merged with
    those of the target."""
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                            (p["blur_kernel_px"],) * 2, 0).astype(np.float32)
    # The background is smooth: blur a downscaled copy (much faster) and scale it back
    scale = p["background_downscale"]
    small = cv2.resize(gray, None, fx=1.0 / scale, fy=1.0 / scale, interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), p["background_sigma_px"] / scale)
    ratio = gray / np.maximum(cv2.resize(small, gray.shape[::-1]), 1.0)
    if p["segmentation"] == "dark":
        binary = (ratio < p["dark_threshold"]).astype(np.uint8) * 255
    else:
        binary = (np.abs(ratio - 1.0) > p["threshold"]).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p["morph_kernel_px"],) * 2)
    mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=p["close_iterations"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = mask.shape
    m = p["border_margin_px"]
    candidates = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if (x > m and y > m and x + w < width - m and y + h < height - m
                and cv2.contourArea(contour) >= p["min_area_px"]):
            hull = cv2.convexHull(contour)
            candidates.append(Candidate(hull, fit_polygon(hull, p["sides"], p["angle_step_deg"]),
                                        width_ratio(hull, p["angle_step_deg"])))
    return binary, candidates


def draw(image: np.ndarray, chosen: Optional[Candidate], objects: list, principal_point: tuple,
         lines: list, mm_per_px: float, inner_circle_mm: float) -> np.ndarray:
    """Image centre, robot axes (bottom right), the objects (red rejected, orange not chosen,
    numbered by rank), the chosen polygon green, and the text lines top right."""
    out = image.copy()
    height, width = out.shape[:2]
    corner = (width - 40, height - 40)  # x_robo = -y_cam (up), y_robo = -x_cam (left)
    for label, tip in (("x_R", (corner[0], corner[1] - 70)), ("y_R", (corner[0] - 70, corner[1]))):
        cv2.arrowedLine(out, corner, tip, GREEN, 2, tipLength=0.2)
        cv2.putText(out, label, (tip[0] - 45, tip[1] - 8), FONT, 0.7, GREEN, 2)
    pp = tuple(int(round(c)) for c in principal_point)
    cv2.drawMarker(out, pp, YELLOW, cv2.MARKER_CROSS, 40, 2)
    for c in objects:
        colour = GREEN if c is chosen else RED if c.verdict == REJECTED else ORANGE
        if c is not chosen:
            cv2.polylines(out, [c.hull], True, colour, 2)
        label = f"{c.polygon.across_flats_px * mm_per_px:.1f} mm w {c.width_ratio:.2f}"
        x, y = cv2.boundingRect(c.hull)[:2]
        cv2.putText(out, f"#{c.rank} {label}" if c.rank else label, (x, max(y - 8, 20)), FONT,
                    0.7, colour, 2)
    if chosen is not None:
        centre = tuple(int(round(v)) for v in chosen.polygon.centre)
        cv2.polylines(out, [np.round(chosen.polygon.corners()).astype(np.int32)], True, GREEN, 2)
        if inner_circle_mm > 0:  # stud: the thread inside the square base
            cv2.circle(out, centre, int(round(inner_circle_mm / 2.0 / mm_per_px)), GREEN, 2)
        cv2.drawMarker(out, centre, GREEN, cv2.MARKER_CROSS, 20, 2)
        cv2.line(out, pp, centre, GREEN, 1)
    for i, line in enumerate(lines):
        size = cv2.getTextSize(line, FONT, 1.0, 2)[0]
        cv2.putText(out, line, (width - 40 - size[0], 50 + 40 * i), FONT, 1.0,
                    GREEN if chosen is not None else RED, 2)
    return out


def tile(image: np.ndarray, title: str, scale: float) -> np.ndarray:
    out = cv2.resize(image if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR),
                     None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.putText(out, title, (10, 30), FONT, 0.8, (0, 0, 0), 4)
    cv2.putText(out, title, (10, 30), FONT, 0.8, YELLOW, 2)
    return out


def debug_view(image: np.ndarray, binary: np.ndarray, result: np.ndarray, caption: str,
               scale: float) -> np.ndarray:
    """Top: camera image | binary mask; below: the result at twice the scale, as wide."""
    top = np.hstack([tile(image, "camera", scale), tile(binary, caption, scale)])
    bottom = tile(result, "result", 2.0 * scale)
    bottom = cv2.resize(bottom, (top.shape[1], round(bottom.shape[0] * top.shape[1]
                                                     / bottom.shape[1])))
    return np.vstack([top, bottom])


class VisionNode(Node):
    def __init__(self):
        # All parameters come from vision.yaml (a missing one raises KeyError when used)
        super().__init__("vision_node", automatically_declare_parameters_from_overrides=True)
        t = self.group("tracking")
        self.tracker = Tracker(t["frames"], t["match_radius_px"], t["max_misses"])
        self.reset_tracks, self.locked = True, None
        self.bridge = CvBridge()
        self.k = self.dist = None
        self.add_on_set_parameters_callback(self.on_parameters)
        self.pub = self.create_publisher(Detection, "vision/detection", 10)
        self.debug_pub = self.create_publisher(Image, "vision/debug", 1)
        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)

    def group(self, prefix: str) -> dict:
        return {name: p.value for name, p in self.get_parameters_by_prefix(prefix).items()}

    def value(self, name: str):
        return self.get_parameter(name).value

    def on_parameters(self, parameters):
        # Each request of sa_tasks (target, distance, lock) starts the tracks over
        if any(p.name in ("target", "object_plane_distance_mm", "lock") for p in parameters):
            self.reset_tracks = True
        return SetParametersResult(successful=True)

    def on_info(self, msg):
        self.k = np.reshape(msg.k, (3, 3))
        self.dist = np.asarray(msg.d, float)

    def undistort(self, points) -> np.ndarray:
        return cv2.undistortPoints(np.reshape(points, (-1, 1, 2)).astype(float), self.k,
                                   self.dist, P=self.k)[:, 0]

    def on_image(self, msg):
        if self.k is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        target, lock = self.value("target"), self.value("lock")
        p = {**self.group("detection"), **self.group(f"targets.{target}")}
        d_mm = self.value("object_plane_distance_mm")
        mm_per_px = d_mm / self.k[0, 0]
        target_px = p["target_size_mm"] / mm_per_px
        binary, objects = find_objects(image, p)
        status = choose(objects, p, target_px) if objects and target == "nut" else ""
        if self.reset_tracks:
            self.tracker.reset()
            self.reset_tracks, self.locked = False, None
        objects = [c for c in self.tracker.update(objects)
                   if c.seen >= self.tracker.frames or not lock]
        ranked = rank(objects, p, target_px, mm_per_px) if target == "stud" else []
        chosen = next((c for c in objects if c.verdict == CHOSEN), None)
        status = status if target == "nut" and objects else f"{len(objects)} object(s)"
        if (lock and self.locked is None and chosen is not None and self.tracker.frames_since_reset
                >= self.value("tracking.lock_after_frames")):
            self.locked = (chosen, objects, ranked)
        if self.locked is not None:
            chosen, objects, ranked = self.locked

        out = Detection(header=msg.header, valid=chosen is not None, status=status,
                        locked=self.locked is not None,
                        frames_since_reset=self.tracker.frames_since_reset,
                        frames_window=self.tracker.frames)
        out.u_undistorted = out.v_undistorted = float("nan")
        lines = [f"z = {d_mm:.0f} mm"]
        if chosen is not None:
            u, v = self.undistort(chosen.polygon.centre)[0]
            out.u_undistorted, out.v_undistorted = float(u), float(v)
            ray = np.linalg.solve(self.k, [u, v, 1.0]) * d_mm  # x_robo = -y_C, y_robo = -x_C
            lines.insert(0, f"x {-ray[1]:+.1f}  y {-ray[0]:+.1f} mm")
        if ranked:
            for c, (u, v) in zip(ranked, self.undistort([c.polygon.centre for c in ranked])):
                out.candidates.append(CandidateMsg(
                    u_undistorted=float(u), v_undistorted=float(v),
                    size_mm=float(c.polygon.across_flats_px * mm_per_px),
                    width_ratio=c.width_ratio, shape_ok=c.shape_ok))
        self.pub.publish(out)

        result = draw(image, chosen, objects, (self.k[0, 2], self.k[1, 2]),
                      lines if chosen is not None else [f"{target}: {status}"] + lines,
                      mm_per_px, p["inner_circle_mm"])
        if lock:
            label, colour = ("LOCKED", GREEN) if self.locked is not None else ("tracking", YELLOW)
            cv2.putText(result, label, (20, result.shape[0] - 20), FONT, 1.5, (0, 0, 0), 8)
            cv2.putText(result, label, (20, result.shape[0] - 20), FONT, 1.5, colour, 3)
        caption = (f"darker than {p['dark_threshold']:.2f} x table" if p["segmentation"] == "dark"
                   else f"table 1 +- {p['threshold']:.2f} removed")
        debug = self.bridge.cv2_to_imgmsg(debug_view(image, binary, result, caption,
                                                     self.value("debug_steps_scale")), "bgr8")
        debug.header = msg.header
        self.debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(VisionNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
