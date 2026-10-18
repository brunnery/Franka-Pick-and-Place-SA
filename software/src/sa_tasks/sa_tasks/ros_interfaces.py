"""ROS implementations of the Robot and Vision interfaces (cartesian_impedance_controller,
franka_robot_state_broadcaster, sa_vision/vision_node).

The node is spun in a background thread; the sequences call these methods from the
main thread and block.
"""
import logging
import threading
import time
from typing import List, Optional

import numpy as np
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sa_interfaces.msg import Detection
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int16

from sa_tasks import config, frames
from sa_tasks.interfaces import Observation
from sa_tasks.trajectory import Waypoint
from sa_vision import geometry

log = logging.getLogger("sa_tasks")

M_TO_MM = 1000.0
POLL_S = 0.01
ARRIVAL_POLL_S = 0.05  # long enough for a meaningful speed estimate from two poses
FORCE_BASELINE_SAMPLES = 20


def pose_msg_to_matrix(pose) -> np.ndarray:
    """geometry_msgs/Pose (m) -> 4x4 (mm)."""
    q = pose.orientation
    p = pose.position
    return frames.pose(frames.quaternion_to_matrix([q.x, q.y, q.z, q.w]),
                       [p.x * M_TO_MM, p.y * M_TO_MM, p.z * M_TO_MM])


def matrix_to_pose_msg(t: np.ndarray) -> Pose:
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = (t[:3, 3] / M_TO_MM).tolist()
    q = frames.matrix_to_quaternion(t[:3, :3])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q.tolist()
    return msg


class RobotState:
    """Latest robot state: end-effector pose, flange -> EE transform, external force."""

    def __init__(self, node: Node, topic: str, r_link8_flange: np.ndarray):
        self.r_link8_flange = r_link8_flange
        self._lock = threading.Lock()
        self._msg: Optional[FrankaRobotState] = None
        node.create_subscription(FrankaRobotState, topic, self._on_state, 10)

    def _on_state(self, msg: FrankaRobotState) -> None:
        with self._lock:
            self._msg = msg

    def wait(self, timeout_s: float) -> FrankaRobotState:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._msg is not None:
                    return self._msg
            time.sleep(POLL_S)
        raise RuntimeError("no robot state received: is the franka bringup running?")

    def flange_and_ee(self, timeout_s: float):
        """(T_B_F, T_F_E) in mm, F with the hand axes (libfranka's f_t_ee is fr3_link8)."""
        msg = self.wait(timeout_s)
        return frames.flange_from_state(pose_msg_to_matrix(msg.o_t_ee.pose),
                                        pose_msg_to_matrix(msg.f_t_ee.pose),
                                        self.r_link8_flange)

    def force_z(self, timeout_s: float) -> float:
        return float(self.wait(timeout_s).o_f_ext_hat_k.wrench.force.z)


class FrankaRobot:
    """Streams flange poses as end-effector references to cartesian_impedance_controller."""

    def __init__(self, node: Node, cfg: dict):
        robot = cfg["robot"]
        self.cfg = robot
        self.state = RobotState(node, robot["state_topic"], config.link8_to_flange(cfg))
        self.reference_pub = node.create_publisher(Pose, robot["reference_topic"], 10)
        self.mode_pub = node.create_publisher(Int16, robot["mode_topic"], 10)
        self.timeout = robot["state_timeout_s"]

    def check_controller(self) -> None:
        deadline = time.monotonic() + self.timeout
        while self.reference_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError(f"nobody listens on {self.cfg['reference_topic']}: "
                                   "is the impedance controller running (sa_robot)?")
            time.sleep(POLL_S)
        self.state.wait(self.timeout)

    def flange_pose(self) -> np.ndarray:
        return self.state.flange_and_ee(self.timeout)[0]

    def force_z(self) -> float:
        return self.state.force_z(self.timeout)

    def set_mode(self, mode: int) -> None:
        self.mode_pub.publish(Int16(data=int(mode)))

    def _publish(self, t_base_flange: np.ndarray, t_flange_ee: np.ndarray) -> None:
        self.reference_pub.publish(
            matrix_to_pose_msg(frames.end_effector_from_flange(t_base_flange, t_flange_ee)))

    def follow(self, path: List[Waypoint], stop_on_contact_n: Optional[float] = None) -> bool:
        """Publish the path in real time. With stop_on_contact_n, stop at the first pose
        where |Fz - Fz before the path| exceeds it: the reference is then set to the
        measured pose and True is returned."""
        _, t_flange_ee = self.state.flange_and_ee(self.timeout)
        baseline = None
        if stop_on_contact_n is not None:
            samples = []
            for _ in range(FORCE_BASELINE_SAMPLES):
                samples.append(self.force_z())
                time.sleep(POLL_S)
            baseline = float(np.median(samples))
        start = time.monotonic()
        for waypoint in path:
            delay = start + waypoint.time_s - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            self._publish(waypoint.t_base_flange, t_flange_ee)
            if baseline is not None and abs(self.force_z() - baseline) > stop_on_contact_n:
                self._publish(self.flange_pose(), t_flange_ee)
                return True
        return False

    def wait_until_arrived(self, goal: np.ndarray, tolerance_mm: float) -> None:
        """Arrived = standing still (speed below arrival_speed_mm_s for arrival_settle_s)
        within tolerance_mm of the goal. The impedance controller keeps a few mm of
        steady-state error; the vision uses the measured pose, so that is harmless."""
        settle, timeout = self.cfg["arrival_settle_s"], self.cfg["arrival_timeout_s"]
        max_speed = self.cfg["arrival_speed_mm_s"]
        deadline = time.monotonic() + timeout
        still_since = None
        last_position, last_time = None, None
        while True:
            position = self.flange_pose()[:3, 3]
            now = time.monotonic()
            error = position - goal[:3, 3]
            speed = (np.inf if last_position is None
                     else float(np.linalg.norm(position - last_position)) / max(now - last_time, 1e-3))
            last_position, last_time = position, now
            if float(np.linalg.norm(error)) <= tolerance_mm and speed <= max_speed:
                still_since = still_since or now
                if now - still_since >= settle:
                    log.info("arrived, remaining error x %+.1f y %+.1f z %+.1f mm", *error)
                    return
            else:
                still_since = None
            if now > deadline:
                raise RuntimeError(f"goal not reached within {timeout} s (error x {error[0]:+.1f} "
                                   f"y {error[1]:+.1f} z {error[2]:+.1f} mm, speed {speed:.1f} mm/s)")
            time.sleep(ARRIVAL_POLL_S)


class RosVision:
    """Asks vision_node for a target and waits for a settled detection taken afterwards."""

    def __init__(self, node: Node, cfg: dict):
        vision = cfg["vision"]
        self.cfg = vision
        self.node = node
        self._lock = threading.Lock()
        self._latest: Optional[Detection] = None
        self._info: Optional[CameraInfo] = None
        node.create_subscription(Detection, vision["detection_topic"], self._on_detection, 10)
        node.create_subscription(CameraInfo, vision["camera_info_topic"], self._on_info, 10)
        self.params = node.create_client(SetParameters, f"{vision['node']}/set_parameters")

    def _on_detection(self, msg: Detection) -> None:
        with self._lock:
            self._latest = msg

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._info = msg

    def _set_parameters(self, target: str, plane_distance_mm: float) -> None:
        if not self.params.wait_for_service(timeout_sec=self.cfg["timeout_s"]):
            raise RuntimeError(f"{self.cfg['node']} not running (sa_vision)")
        values = [
            ("target", ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=target)),
            ("d_source", ParameterValue(type=ParameterType.PARAMETER_STRING, string_value="yaml")),
            ("object_plane_distance_mm",
             ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=plane_distance_mm)),
        ]
        request = SetParameters.Request(
            parameters=[ParameterMsg(name=name, value=value) for name, value in values])
        future = self.params.call_async(request)
        deadline = time.monotonic() + self.cfg["timeout_s"]
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("vision_node did not accept the parameters")
            time.sleep(POLL_S)
        failed = [r.reason for r in future.result().results if not r.successful]
        if failed:
            raise RuntimeError(f"vision_node refused parameters: {failed}")

    def observe(self, target: str, plane_distance_mm: float) -> Observation:
        self._set_parameters(target, plane_distance_mm)
        # Only frames taken after the robot has rung down count
        earliest = self.node.get_clock().now().nanoseconds + int(self.cfg["settle_s"] * 1e9)
        deadline = time.monotonic() + self.cfg["settle_s"] + self.cfg["timeout_s"]
        last_status = "no detection message"
        while time.monotonic() < deadline:
            with self._lock:
                msg, info = self._latest, self._info
            if msg is not None:
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if stamp >= earliest:
                    last_status = msg.status
                    if msg.valid and info is not None:
                        return Observation(msg.u_undistorted, msg.v_undistorted,
                                           geometry.camera_matrix(info.k),
                                           (info.width, info.height))
            time.sleep(POLL_S)
        raise RuntimeError(f"no {target} found within {self.cfg['timeout_s']} s "
                           f"(last status: {last_status})")
