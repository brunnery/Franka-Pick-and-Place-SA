"""Simple object finder: one object on a blank background -> centre of a fitted hexagon.

  ros2 launch software/launch/simple_vision.launch.py

No size or shape filter. Steps: greyscale -> blur -> background = heavily blurred
image -> foreground where |I - B| / B > threshold -> close/open -> largest blob that
does not touch the image border (robot parts reaching into the view, a cut object) ->
regular hexagon fitted to its convex hull -> centre of the hexagon.

Hexagon fit: for each orientation theta (0-60 deg), the hull is enclosed by three
pairs of parallel lines with normals theta, theta+60, theta+120. Each pair gives a
width and a midline. The orientation with the smallest total width gives the
tightest enclosing hexagon (it touches the flats of a hexagonal object); its centre
is the least-squares intersection of the three midlines.

Publishes sa_interfaces/Detection on vision/detection (same topic and node name as
sa_vision, so sa_tasks works with either) and an overlay on vision/debug.
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


def find_object(image: np.ndarray, p: dict) -> tuple:
    """(hexagon or None, foreground mask, status). p: the "detection" parameters."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (p["blur_kernel_px"],) * 2, 0).astype(np.float32)
    # The background is smooth: blur a downscaled copy (much faster) and scale it back
    scale = p["background_downscale"]
    small = cv2.resize(gray, None, fx=1.0 / scale, fy=1.0 / scale, interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), p["background_sigma_px"] / scale)
    background = cv2.resize(small, gray.shape[::-1], interpolation=cv2.INTER_LINEAR)
    difference = np.abs(gray - background) / np.maximum(background, 1.0)
    mask = (difference > p["threshold"]).astype(np.uint8) * 255
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
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < p["min_area_px"]:
        return None, mask, "nothing found (only specks)"
    return fit_hexagon(cv2.convexHull(largest), p["angle_step_deg"]), mask, "ok"


def draw(image: np.ndarray, hexagon: Optional[Hexagon], mask: np.ndarray, status: str,
         principal_point: tuple, info: tuple = ()) -> np.ndarray:
    """Overlay; info = extra text lines (bottom left)."""
    out = image.copy()
    for i, line in enumerate(reversed(info)):
        cv2.putText(out, line, (20, out.shape[0] - 20 - 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    OVERLAY_YELLOW, 2)
    out[mask > 0] = (0.6 * out[mask > 0] + 0.4 * np.array(OVERLAY_YELLOW)).astype(np.uint8)
    pp = tuple(int(round(c)) for c in principal_point)
    cv2.drawMarker(out, pp, OVERLAY_YELLOW, cv2.MARKER_CROSS, 40, 2)
    if hexagon is None:
        cv2.putText(out, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, OVERLAY_RED, 2)
        return out
    corners = np.round(hexagon.corners()).astype(np.int32)
    centre = tuple(int(round(c)) for c in hexagon.centre)
    cv2.polylines(out, [corners], True, OVERLAY_GREEN, 2)
    cv2.drawMarker(out, centre, OVERLAY_GREEN, cv2.MARKER_CROSS, 20, 2)
    cv2.line(out, pp, centre, OVERLAY_GREEN, 1)
    cv2.putText(out, f"u {hexagon.centre[0]:.1f} v {hexagon.centre[1]:.1f}  "
                f"s {hexagon.across_flats_px:.0f} px", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, OVERLAY_GREEN, 2)
    return out


DETECTION_PARAMETERS = ("blur_kernel_px", "background_sigma_px", "background_downscale", "threshold",
                        "morph_kernel_px", "close_iterations", "min_area_px", "border_margin_px",
                        "angle_step_deg")


class SimpleVisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        for name in DETECTION_PARAMETERS:
            kind = Parameter.Type.DOUBLE if name in ("background_sigma_px", "threshold",
                                                     "angle_step_deg") else Parameter.Type.INTEGER
            self.declare_parameter(f"detection.{name}", kind)
        self.declare_parameter("publish_debug", Parameter.Type.BOOL)
        # Set by sa_tasks; only used for point_camera (no size filter here)
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
        hexagon, mask, status = find_object(image, p)
        out = Detection(header=msg.header, valid=hexagon is not None, status=status)
        nan = float("nan")
        out.u = out.v = out.u_undistorted = out.v_undistorted = nan
        out.point_camera.x = out.point_camera.y = out.point_camera.z = nan
        out.across_flats_mm = out.area_px = out.hole_area_px = nan
        out.width_ratio = out.compactness = nan
        d_mm = self.get_parameter("object_plane_distance_mm").value
        out.plane_distance = d_mm * MM_TO_M
        info = [f"z: d = {d_mm:.0f} mm (camera -> object plane, set by sa_tasks)"]
        if hexagon is not None:
            out.u, out.v = hexagon.centre
            und = cv2.undistortPoints(np.array([[hexagon.centre]], float), self.k, self.dist,
                                      P=self.k)[0, 0]
            out.u_undistorted, out.v_undistorted = float(und[0]), float(und[1])
            ray = np.linalg.solve(self.k, [und[0], und[1], 1.0]) * d_mm * MM_TO_M
            out.point_camera.x, out.point_camera.y, out.point_camera.z = ray.tolist()
            out.across_flats_mm = hexagon.across_flats_px * d_mm / self.k[0, 0]
            # Robot axes: x_robo = -y_cam, y_robo = -x_cam (CLAUDE.md)
            info.append(f"offset from image centre: x_robo {-ray[1] / MM_TO_M:+.1f} mm, "
                        f"y_robo {-ray[0] / MM_TO_M:+.1f} mm")
            out.width_ratio = hexagon.regularity
            out.num_candidates = 1
        self.pub.publish(out)
        if self.get_parameter("publish_debug").value:
            debug = self.bridge.cv2_to_imgmsg(
                draw(image, hexagon, mask, status, (self.k[0, 2], self.k[1, 2]), tuple(info)),
                "bgr8")
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
