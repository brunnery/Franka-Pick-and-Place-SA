"""ROS implementations of the Robot and Vision interfaces (cartesian_impedance_controller,
franka_robot_state_broadcaster, sa_vision/vision_node).

The node is spun in a background thread; the sequences call these methods and block.
"""
import logging
import time
from typing import Callable, List, Optional

import cv2
import numpy as np
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sa_interfaces.msg import Detection
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int16

from sa_tasks import config, frames
from sa_tasks.interfaces import Candidate, Observation, View
from sa_tasks.trajectory import Waypoint

log = logging.getLogger("sa_tasks")

M_TO_MM = 1000.0
POLL_S = 0.01
ARRIVAL_POLL_S = 0.05  # long enough for a meaningful speed estimate from two poses
FORCE_BASELINE_SAMPLES = 20
ROBOT_MODES = {0: "other", 1: "idle", 2: "move", 3: "guiding", 4: "reflex", 5: "user stopped",
               6: "automatic error recovery"}  # FrankaRobotState.robot_mode
PARAMETER_KINDS = {str: ("string_value", ParameterType.PARAMETER_STRING),
                   bool: ("bool_value", ParameterType.PARAMETER_BOOL),
                   float: ("double_value", ParameterType.PARAMETER_DOUBLE)}


def pose_msg_to_matrix(pose) -> np.ndarray:
    """geometry_msgs/Pose (m) -> 4x4 (mm)."""
    q, p = pose.orientation, pose.position
    return frames.pose(frames.quaternion_to_matrix([q.x, q.y, q.z, q.w]),
                       [p.x * M_TO_MM, p.y * M_TO_MM, p.z * M_TO_MM])


def matrix_to_pose_msg(t: np.ndarray) -> Pose:
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = (t[:3, 3] / M_TO_MM).tolist()
    q = frames.matrix_to_quaternion(t[:3, :3])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q.tolist()
    return msg


def wait_for(get, timeout_s: float, what: str):
    """Poll get() until it returns something other than None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = get()
        if value is not None:
            return value
        time.sleep(POLL_S)
    raise RuntimeError(f"{what} within {timeout_s} s")


class RobotState:
    """Latest robot state: flange pose, flange -> EE transform, external force."""

    def __init__(self, node: Node, topic: str, r_link8_flange: np.ndarray, timeout_s: float):
        self.r_link8_flange = r_link8_flange
        self.timeout = timeout_s
        self.msg: Optional[FrankaRobotState] = None
        node.create_subscription(FrankaRobotState, topic, lambda m: setattr(self, "msg", m), 10)

    def latest(self) -> FrankaRobotState:
        return wait_for(lambda: self.msg, self.timeout, "no robot state (franka bringup?)")

    def flange_and_ee(self):
        """(T_B_F, T_F_E) in mm, F with the hand axes (libfranka's f_t_ee is fr3_link8)."""
        msg = self.latest()
        return frames.flange_from_state(pose_msg_to_matrix(msg.o_t_ee.pose),
                                        pose_msg_to_matrix(msg.f_t_ee.pose), self.r_link8_flange)


class FrankaRobot:
    """Streams flange poses as end-effector references to cartesian_impedance_controller."""

    def __init__(self, node: Node, cfg: dict):
        self.cfg = cfg["robot"]
        self.state = RobotState(node, self.cfg["state_topic"], config.link8_to_flange(cfg),
                                self.cfg["state_timeout_s"])
        self.reference_pub = node.create_publisher(Pose, self.cfg["reference_topic"], 10)
        self.mode_pub = node.create_publisher(Int16, self.cfg["mode_topic"], 10)
        self.arrivals: List[list] = []  # remaining error [x, y, z] mm of every arrival
        wait_for(lambda: self.reference_pub.get_subscription_count() or None,
                 self.cfg["state_timeout_s"], "no impedance controller listening")
        # A start move of the controller after activation swallowed the first references
        # (20.10.): begin only once the arm stands still
        self.wait_until_arrived(None, 0.0, self.cfg["start_settle_s"])

    def flange_pose(self) -> np.ndarray:
        return self.state.flange_and_ee()[0]

    def force_z(self) -> float:
        return float(self.state.latest().o_f_ext_hat_k.wrench.force.z)

    def set_mode(self, mode: int) -> None:
        self.mode_pub.publish(Int16(data=int(mode)))

    def follow(self, path: List[Waypoint], stop_on_contact_n: Optional[float] = None,
               contact_from_s: float = 0.0,
               stop_when: Optional[Callable[[], bool]] = None) -> bool:
        """Publish the path in real time. With stop_on_contact_n, stop at the first pose
        from time contact_from_s on where |Fz - Fz before the path| exceeds it: the reference
        is then set to the measured pose and True is returned. (Earlier in a fast move the
        force estimate also sees the arm's own acceleration.) With stop_when, stop as soon as
        it returns True (checked after every pose); the last reference stays, True is
        returned."""
        t_flange_ee = self.state.flange_and_ee()[1]
        baseline = None
        if stop_on_contact_n is not None:
            samples = []
            for _ in range(FORCE_BASELINE_SAMPLES):
                samples.append(self.force_z())
                time.sleep(POLL_S)
            baseline = float(np.median(samples))
        start = time.monotonic()
        for waypoint in path:
            time.sleep(max(start + waypoint.time_s - time.monotonic(), 0.0))
            self.reference_pub.publish(matrix_to_pose_msg(waypoint.t_base_flange @ t_flange_ee))
            if (baseline is not None and waypoint.time_s >= contact_from_s
                    and abs(self.force_z() - baseline) > stop_on_contact_n):
                self.reference_pub.publish(matrix_to_pose_msg(self.flange_pose() @ t_flange_ee))
                return True
            if stop_when is not None and stop_when():
                return True
        return False

    def wait_until_arrived(self, goal: Optional[np.ndarray], tolerance_mm: float,
                           settle_s: Optional[float] = None,
                           hold: Optional[np.ndarray] = None,
                           speed_mm_s: Optional[float] = None) -> None:
        """Arrived = standing still (below arrival_speed_mm_s, or speed_mm_s, for settle_s)
        within tolerance_mm of the goal (goal None: anywhere). The impedance controller keeps a few
        mm of steady-state error; the vision uses the measured pose, so that is harmless.
        hold: publish this flange reference with every poll. Every reference message resets the
        controller's integral term, so the arm then stands on the spring alone, as it does
        while a path is streamed (25.10.: after the fine positioning the descent wiped the
        integral and the arm gave way by up to ~1 mm)."""
        settle = self.cfg["arrival_settle_s"] if settle_s is None else settle_s
        still = self.cfg["arrival_speed_mm_s"] if speed_mm_s is None else speed_mm_s
        hold_msg = None if hold is None else matrix_to_pose_msg(
            hold @ self.state.flange_and_ee()[1])
        deadline = time.monotonic() + self.cfg["arrival_timeout_s"]
        still_since, last = None, None
        while True:
            if hold_msg is not None:
                self.reference_pub.publish(hold_msg)
            position, now = self.flange_pose()[:3, 3], time.monotonic()
            error = np.zeros(3) if goal is None else position - goal[:3, 3]
            speed = np.inf if last is None else np.linalg.norm(position - last[0]) / (now - last[1])
            last = (position, now)
            if np.linalg.norm(error) <= tolerance_mm and speed <= still:
                still_since = still_since or now
                if now - still_since >= settle:
                    if goal is not None:
                        log.info("arrived, remaining error x %+.1f y %+.1f z %+.1f mm", *error)
                        self.arrivals.append([round(float(e), 2) for e in error])
                    return
            else:
                still_since = None
            if now > deadline:
                mode = self.state.latest().robot_mode
                raise RuntimeError(f"not arrived (error {np.round(error, 1).tolist()} mm, "
                                   f"{speed:.1f} mm/s, robot mode {ROBOT_MODES.get(mode, mode)})")
            time.sleep(ARRIVAL_POLL_S)


def image_to_bgr(msg: Image) -> np.ndarray:
    """rgb8/bgr8 sensor_msgs/Image as an OpenCV BGR array."""
    image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    image = image.reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else image.copy()


class RosVision:
    """Asks vision_node for a target and waits for a detection taken after the request.
    Keeps the camera and debug image of every answered request (snapshots) for the record."""

    def __init__(self, node: Node, cfg: dict):
        self.cfg = cfg["vision"]
        self.node = node
        self.detection: Optional[Detection] = None
        self.info: Optional[CameraInfo] = None
        self.images = {}  # latest message per topic key
        self.snapshots: List[tuple] = []  # (label, {key: Image})
        for key in ("raw", "debug"):
            node.create_subscription(Image, self.cfg[f"{key}_image_topic"],
                                     lambda m, k=key: self.images.__setitem__(k, m),
                                     qos_profile_sensor_data)
        node.create_subscription(Detection, self.cfg["detection_topic"],
                                 lambda m: setattr(self, "detection", m), 10)
        node.create_subscription(CameraInfo, self.cfg["camera_info_topic"],
                                 lambda m: setattr(self, "info", m), 10)
        self.params = node.create_client(SetParameters, f"{self.cfg['node']}/set_parameters")

    def prepare(self, target: str) -> None:
        """Select the target early, so the overlay shows the right polygon."""
        self._set({"target": target})

    def observe(self, target: str, plane_distance_mm: float, rough: bool = False) -> Observation:
        msg, k = self._settled(target, plane_distance_mm, False,
                               lambda m: m.valid and m.frames_since_reset >= m.frames_window,
                               rough)
        self._snapshot(target)
        return Observation(msg.u_undistorted, msg.v_undistorted, k)

    def view(self, target: str, plane_distance_mm: float, lock: bool,
             rough: bool = False) -> View:
        """lock: the objects vision_node has locked on (none once empty_view_frames images
        showed nothing); else the medians once a full window of images has been taken."""
        def accept(m):
            if lock:
                return m.locked or m.frames_since_reset >= self.cfg["empty_view_frames"]
            return m.frames_since_reset >= m.frames_window
        msg, k = self._settled(target, plane_distance_mm, lock, accept, rough)
        self._snapshot(f"{target}_{'observe' if lock else 'centred'}")
        return View([Candidate(c.u_undistorted, c.v_undistorted, c.size_mm, c.width_ratio,
                               c.shape_ok) for c in msg.candidates], k)

    def snapshot(self, label: str) -> None:
        """Keep the current camera and debug image for the record, no detection (e.g. the
        result at the end of a run). Waits settle_s first, as a detection does."""
        time.sleep(self.cfg["settle_s"])
        self._snapshot(label)

    def _snapshot(self, label: str) -> None:
        self.snapshots.append((label, dict(self.images)))

    def save_snapshots(self, prefix) -> None:
        """<prefix>_<n>_<label>_<raw|debug>.png per answered request."""
        for number, (label, images) in enumerate(self.snapshots, 1):
            for key, msg in images.items():
                path = f"{prefix}_{number}_{label}_{key}.png"
                cv2.imwrite(path, image_to_bgr(msg))

    def _set(self, values: dict) -> None:
        if not self.params.wait_for_service(timeout_sec=self.cfg["timeout_s"]):
            raise RuntimeError(f"{self.cfg['node']} not running (sa_vision)")
        parameters = []
        for name, value in values.items():
            field, kind = PARAMETER_KINDS[type(value)]
            parameters.append(ParameterMsg(name=name,
                                           value=ParameterValue(type=kind, **{field: value})))
        future = self.params.call_async(SetParameters.Request(parameters=parameters))
        wait_for(lambda: future.result() if future.done() else None, self.cfg["timeout_s"],
                 "vision_node did not accept the parameters")
        failed = [r.reason for r in future.result().results if not r.successful]
        if failed:
            raise RuntimeError(f"vision_node refused parameters: {failed}")

    def _settled(self, target: str, plane_distance_mm: float, lock: bool, accept,
                 rough: bool = False) -> tuple:
        """First detection after the request for which accept(msg). The request is sent only
        after settle_s (robot rung down; rough: rough_settle_s); it starts the tracks of
        vision_node over, so no image of the motion is in the median."""
        time.sleep(self.cfg["rough_settle_s" if rough else "settle_s"])
        self._set({"target": target, "lock": lock,
                   "object_plane_distance_mm": float(plane_distance_mm)})
        earliest = self.node.get_clock().now().nanoseconds

        def settled():
            msg, info = self.detection, self.info
            if msg is None or info is None:
                return None
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            return (msg, np.reshape(info.k, (3, 3))) if stamp >= earliest and accept(msg) else None
        try:
            return wait_for(settled, self.cfg["timeout_s"], "")
        except RuntimeError:
            status = getattr(self.detection, "status", "no detection message")
            raise RuntimeError(f"no {target} found within {self.cfg['timeout_s']} s "
                               f"(last status: {status})") from None
