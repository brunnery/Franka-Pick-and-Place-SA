"""ROS 2 node: nut detection on the camera stream.

Subscribes:  image_raw (sensor_msgs/Image), camera_info (sensor_msgs/CameraInfo)
Publishes:   vision/detection (sa_interfaces/Detection), vision/debug (sensor_msgs/Image)
All parameters come from config/vision.yaml (no defaults in code).
"""
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from sa_interfaces.msg import Detection
from sa_vision import geometry
from sa_vision.debug_draw import draw_debug
from sa_vision.detection import NutParams, detect_nut

MM_TO_M = 1e-3

DETECTION_PARAMETERS = {
    "blur_kernel_px": Parameter.Type.INTEGER,
    "morph_kernel_px": Parameter.Type.INTEGER,
    "morph_open_iterations": Parameter.Type.INTEGER,
    "morph_close_iterations": Parameter.Type.INTEGER,
    "roi": Parameter.Type.INTEGER_ARRAY,
    "across_flats_mm": Parameter.Type.DOUBLE,
    "hole_diameter_mm": Parameter.Type.DOUBLE,
    "area_tolerance": Parameter.Type.DOUBLE,
    "hole_ratio_tolerance": Parameter.Type.DOUBLE,
    "min_compactness": Parameter.Type.DOUBLE,
    "max_centre_offset_ratio": Parameter.Type.DOUBLE,
    "centre_source": Parameter.Type.STRING,
}


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("object_plane_distance_mm", Parameter.Type.DOUBLE)
        self.declare_parameter("publish_debug", Parameter.Type.BOOL)
        for name, kind in DETECTION_PARAMETERS.items():
            self.declare_parameter(f"nut.{name}", kind)

        self.bridge = CvBridge()
        self.k = None
        self.dist = None
        self.detection_pub = self.create_publisher(Detection, "vision/detection", 10)
        # Reliable with depth 1: rqt_image_view subscribes reliable, old frames are useless.
        self.debug_pub = self.create_publisher(Image, "vision/debug", 1)
        self.create_subscription(CameraInfo, "camera_info", self.on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)
        self.get_logger().info("Waiting for camera_info and image_raw")

    def nut_params(self) -> NutParams:
        values = {name: self.get_parameter(f"nut.{name}").value for name in DETECTION_PARAMETERS}
        values["roi"] = tuple(values["roi"])
        return NutParams(**values)

    def on_camera_info(self, msg: CameraInfo):
        if self.k is None:
            self.get_logger().info("Received camera intrinsics")
        self.k = geometry.camera_matrix(msg.k)
        self.dist = list(msg.d)

    def on_image(self, msg: Image):
        if self.k is None:
            return
        distance_mm = self.get_parameter("object_plane_distance_mm").value
        focal_px = 0.5 * (self.k[0, 0] + self.k[1, 1])
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        result = detect_nut(image, focal_px, distance_mm, self.nut_params())

        out = Detection()
        out.header = msg.header
        out.valid = result.valid
        out.status = result.status
        out.u, out.v = result.u, result.v
        out.plane_distance = distance_mm * MM_TO_M
        out.area_px = result.area_px
        out.hole_area_px = result.hole_area_px
        out.expected_area_px = result.expected_area_px
        out.compactness = result.compactness
        out.num_candidates = result.num_accepted
        if result.valid:
            out.u_undistorted, out.v_undistorted = geometry.undistort_pixel(
                result.u, result.v, self.k, self.dist)
            x, y, z = geometry.pixel_to_camera_point(
                out.u_undistorted, out.v_undistorted, self.k, out.plane_distance)
            out.point_camera.x, out.point_camera.y, out.point_camera.z = x, y, z
        else:
            nan = float("nan")
            out.u_undistorted = out.v_undistorted = nan
            out.point_camera.x = out.point_camera.y = out.point_camera.z = nan
        self.detection_pub.publish(out)

        if self.get_parameter("publish_debug").value:
            label = ""
            if result.valid:
                p = out.point_camera
                label = (f"P_C = ({p.x / MM_TO_M:.1f}, {p.y / MM_TO_M:.1f}, "
                         f"{p.z / MM_TO_M:.1f}) mm")
            debug = draw_debug(image, result, (self.k[0, 2], self.k[1, 2]), label)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
