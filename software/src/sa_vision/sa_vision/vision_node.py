"""ROS 2 node: nut detection on the camera stream.

Subscribes:  image_raw (sensor_msgs/Image), camera_info (sensor_msgs/CameraInfo)
Publishes:   vision/detection (sa_interfaces/Detection), vision/debug (sensor_msgs/Image)
All parameters come from config/vision.yaml (no defaults in code).
"""
from pathlib import Path
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from sa_interfaces.msg import Detection
from sa_vision import geometry
from sa_vision.debug_draw import draw_debug, draw_steps
from sa_vision.detection import BACKGROUND_MISMATCH, NutParams, detect_nut
from sa_vision.params import distance_range, nut_params
from sa_vision.temporal import CentreMedian

MM_TO_M = 1e-3
FPS_SMOOTHING = 0.1  # weight of the newest frame interval in the displayed frame rate
DEBUG_VIEWS = ("overlay", "steps")
REFERENCE_WARN_PERIOD_S = 5.0

DETECTION_PARAMETERS = {
    "blur_kernel_px": Parameter.Type.INTEGER,
    "morph_kernel_px": Parameter.Type.INTEGER,
    "morph_close_iterations": Parameter.Type.INTEGER,
    "morph_open_iterations": Parameter.Type.INTEGER,
    "roi": Parameter.Type.INTEGER_ARRAY,
    "segmentation": Parameter.Type.STRING,
    "edge_threshold": Parameter.Type.DOUBLE,
    "edge_group_ratio": Parameter.Type.DOUBLE,
    "reference_match_range": Parameter.Type.DOUBLE_ARRAY,
    "background_downscale": Parameter.Type.INTEGER,
    "background_window_ratio": Parameter.Type.DOUBLE,
    "brighter_threshold": Parameter.Type.DOUBLE,
    "darker_threshold": Parameter.Type.DOUBLE,
    "core_threshold": Parameter.Type.DOUBLE,
    "across_flats_range_mm": Parameter.Type.DOUBLE_ARRAY,
    "target_across_flats_mm": Parameter.Type.DOUBLE,
    "size_tolerance": Parameter.Type.DOUBLE,
    "size_class_tolerance": Parameter.Type.DOUBLE,
    "min_width_ratio": Parameter.Type.DOUBLE,
    "max_width_ratio": Parameter.Type.DOUBLE,
    "min_hole_ratio": Parameter.Type.DOUBLE,
    "max_hole_ratio": Parameter.Type.DOUBLE,
    "max_centre_offset_ratio": Parameter.Type.DOUBLE,
}


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("d_source", Parameter.Type.STRING)
        self.declare_parameter("d_from_size_range_mm", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("object_plane_distance_mm", Parameter.Type.DOUBLE)
        self.declare_parameter("publish_debug", Parameter.Type.BOOL)
        self.declare_parameter("debug_view", Parameter.Type.STRING)
        self.declare_parameter("debug_steps_scale", Parameter.Type.DOUBLE)
        self.declare_parameter("centre_median_frames", Parameter.Type.INTEGER)
        self.declare_parameter("centre_reset_jump_ratio", Parameter.Type.DOUBLE)
        self.declare_parameter("reference_image", Parameter.Type.STRING)
        for name, kind in DETECTION_PARAMETERS.items():
            self.declare_parameter(f"nut.{name}", kind)

        self.background = self.load_reference(self.get_parameter("reference_image").value)
        self.bridge = CvBridge()
        self.distance_range()  # an unknown d_source fails at start, not per frame
        if self.get_parameter("debug_view").value not in DEBUG_VIEWS:
            raise ValueError(f"debug_view must be one of {DEBUG_VIEWS}")
        self.centre_filter = CentreMedian(self.get_parameter("centre_median_frames").value,
                                          self.get_parameter("centre_reset_jump_ratio").value)
        self.k = None
        self.dist = None
        self.last_frame_time = None
        self.fps = None
        self.detection_pub = self.create_publisher(Detection, "vision/detection", 10)
        # Reliable with depth 1: rqt_image_view subscribes reliable, old frames are useless.
        self.debug_pub = self.create_publisher(Image, "vision/debug", 1)
        self.create_subscription(CameraInfo, "camera_info", self.on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)
        self.get_logger().info("Waiting for camera_info and image_raw")

    def load_reference(self, path: str):
        """Reference image of the empty work area (method A'), None = estimate (A)."""
        if path and Path(path).is_file():
            image = cv2.imread(path)
            if image is None:
                raise RuntimeError(f"Cannot read reference image {path}")
            self.get_logger().info(f"Background: reference image {Path(path).resolve()}")
            return image
        self.get_logger().warn(f"No reference image at '{path}', only the local median is "
                               "used (method A). Record one with capture_reference.")
        return None

    def distance_range(self):
        """(d_min, d_max) if d is estimated from the nut size, None if d comes from the YAML."""
        return distance_range(self.get_parameter("d_source").value,
                              self.get_parameter("d_from_size_range_mm").value)

    def update_fps(self):
        now = time.monotonic()
        if self.last_frame_time is not None and now > self.last_frame_time:
            rate = 1.0 / (now - self.last_frame_time)
            self.fps = rate if self.fps is None else (
                FPS_SMOOTHING * rate + (1.0 - FPS_SMOOTHING) * self.fps)
        self.last_frame_time = now

    def nut_params(self) -> NutParams:
        return nut_params({name: self.get_parameter(f"nut.{name}").value
                           for name in DETECTION_PARAMETERS})

    def on_camera_info(self, msg: CameraInfo):
        if self.k is None:
            self.get_logger().info("Received camera intrinsics")
        self.k = geometry.camera_matrix(msg.k)
        self.dist = list(msg.d)

    def on_image(self, msg: Image):
        if self.k is None:
            return
        self.update_fps()
        distance_mm = self.get_parameter("object_plane_distance_mm").value
        focal_px = 0.5 * (self.k[0, 0] + self.k[1, 1])
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        params = self.nut_params()
        result = detect_nut(image, focal_px, distance_mm, params, self.background,
                            self.distance_range())
        if result.background == BACKGROUND_MISMATCH:
            self.get_logger().warn(
                f"Reference image ignored: live / reference brightness "
                f"{result.reference_brightness:.2f} (lighting changed?). Record a new one with "
                f"capture_reference.", throttle_duration_sec=REFERENCE_WARN_PERIOD_S)
        filtered = self.centre_filter.update(result)
        settled = filtered is not None and filtered.settled

        # The published centre is the median over the last frames; it is valid only once
        # the window is full (THEORY 2.7).
        out = Detection()
        out.header = msg.header
        out.valid = settled
        out.status = (f"settling {filtered.frames}/{filtered.window}"
                      if filtered is not None and not settled else result.status)
        out.across_flats_mm = result.across_flats_mm
        out.size = result.size
        out.area_px = result.area_px
        out.hole_area_px = result.hole_area_px
        out.width_ratio = result.width_ratio
        out.compactness = result.compactness
        out.num_candidates = result.num_accepted
        nan = float("nan")
        robot_offset = None
        if filtered is not None:
            u_undistorted, v_undistorted = geometry.undistort_pixel(
                filtered.u, filtered.v, self.k, self.dist)
            point = geometry.pixel_to_camera_point(u_undistorted, v_undistorted, self.k,
                                                   filtered.distance_mm * MM_TO_M)
            robot_offset = geometry.camera_to_robot_xy(point[0] / MM_TO_M, point[1] / MM_TO_M)
        if settled:
            out.u, out.v = filtered.u, filtered.v
            out.u_undistorted, out.v_undistorted = u_undistorted, v_undistorted
            out.plane_distance = filtered.distance_mm * MM_TO_M
            out.point_camera.x, out.point_camera.y, out.point_camera.z = point
        else:
            out.u = out.v = out.u_undistorted = out.v_undistorted = nan
            out.plane_distance = result.distance_mm * MM_TO_M  # NaN if estimated
            out.point_camera.x = out.point_camera.y = out.point_camera.z = nan
        self.detection_pub.publish(out)

        if self.get_parameter("publish_debug").value:
            principal_point = (self.k[0, 2], self.k[1, 2])
            window = self.centre_filter.window
            if self.get_parameter("debug_view").value == "steps":
                debug = draw_steps(image, result, principal_point, params,
                                   self.get_parameter("debug_steps_scale").value, self.fps,
                                   robot_offset, filtered, window)
            else:
                debug = draw_debug(image, result, principal_point, self.fps, robot_offset,
                                   filtered, window)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    # Ctrl-C reaches the node twice (terminal and launch), so no explicit
    # destroy_node(): a second SIGINT there would end in a traceback.
    rclpy.init(args=args)
    try:
        rclpy.spin(VisionNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
