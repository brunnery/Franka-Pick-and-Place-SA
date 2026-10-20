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
from sa_tasks.interfaces import Candidate, Observation, View
from sa_tasks.trajectory import Waypoint
from sa_vision import geometry

log = logging.getLogger("sa_tasks")

M_TO_MM = 1000.0
POLL_S = 0.01
ARRIVAL_POLL_S = 0.05  # long enough for a meaningful speed estimate from two poses
FORCE_BASELINE_SAMPLES = 20
# FrankaRobotState.robot_mode, for error messages (2 = move: the controller is in charge)
ROBOT_MODES = {0: "other", 1: "idle", 2: "move", 3: "guiding", 4: "reflex",
               5: "user stopped", 6: "automatic error recovery"}


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
        self.arrivals: List[list] = []  # remaining error [x, y, z] mm of every arrival

    def check_controller(self) -> None:
        deadline = time.monotonic() + self.timeout
        while self.reference_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError(f"nobody listens on {self.cfg['reference_topic']}: "
                                   "is the impedance controller running (sa_robot)?")
            time.sleep(POLL_S)
        self.state.wait(self.timeout)
        self._wait_until_still()

    def _wait_until_still(self) -> None:
        """At activation the controller makes its own start move (controllers.yaml,
        start_pose); references sent during it were not always taken (20.10.: the arm went
        to the start pose instead of the observation pose). So the first move waits until
        the arm has stood still for start_settle_s."""
        settle, timeout = self.cfg["start_settle_s"], self.cfg["arrival_timeout_s"]
        max_speed = self.cfg["arrival_speed_mm_s"]
        deadline = time.monotonic() + timeout
        still_since = None
        last_position, last_time = self.flange_pose()[:3, 3], time.monotonic()
        waited = False
        while True:
            time.sleep(ARRIVAL_POLL_S)
            position, now = self.flange_pose()[:3, 3], time.monotonic()
            speed = float(np.linalg.norm(position - last_position)) / max(now - last_time, 1e-3)
            last_position, last_time = position, now
            if speed <= max_speed:
                still_since = still_since or now
                if now - still_since >= settle:
                    if waited:
                        log.info("controller start move over, arm still")
                    return
            else:
                if not waited:
                    log.info("arm moving (controller start move?): waiting until it stands still")
                waited, still_since = True, None
            if now > deadline:
                raise RuntimeError(f"arm not still within {timeout} s after the controller "
                                   f"start (speed {speed:.1f} mm/s)")

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
                    self.arrivals.append([round(float(e), 2) for e in error])
                    return
            else:
                still_since = None
            if now > deadline:
                mode = self.state.wait(self.timeout).robot_mode
                raise RuntimeError(f"goal not reached within {timeout} s (error x {error[0]:+.1f} "
                                   f"y {error[1]:+.1f} z {error[2]:+.1f} mm, speed {speed:.1f} "
                                   f"mm/s, robot mode: {ROBOT_MODES.get(mode, mode)})")
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

    def prepare(self, target: str) -> None:
        """Select the target in vision_node at the start of a step, so the overlay shows the
        right polygon before the first detection is asked for."""
        self._set_parameters(target)

    def _set_parameters(self, target: str, plane_distance_mm: Optional[float] = None,
                        lock: bool = False) -> None:
        if not self.params.wait_for_service(timeout_sec=self.cfg["timeout_s"]):
            raise RuntimeError(f"{self.cfg['node']} not running (sa_vision)")
        values = [
            ("target", ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=target)),
            ("d_source", ParameterValue(type=ParameterType.PARAMETER_STRING, string_value="yaml")),
            ("lock", ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=lock)),
        ]
        if plane_distance_mm is not None:
            values.append(("object_plane_distance_mm", ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=plane_distance_mm)))
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
        # The nut: median over the last images, no lock (as before 20.10.)
        msg, info = self._settled(target, plane_distance_mm, False,
                                  lambda m: m.valid and m.frames_since_reset >= m.frames_window)
        return Observation(msg.u_undistorted, msg.v_undistorted, geometry.camera_matrix(info.k),
                           (info.width, info.height))

    def view(self, target: str, plane_distance_mm: float, lock: bool = True) -> View:
        """Objects of one view, most likely target first. lock: the objects vision_node has
        locked on (possibly none, once empty_view_frames images showed nothing); else the
        medians once a full window of images has been taken."""
        if lock:
            def accept(m):
                return m.locked or m.frames_since_reset >= self.cfg["empty_view_frames"]
        else:
            def accept(m):
                return m.frames_since_reset >= m.frames_window
        msg, info = self._settled(target, plane_distance_mm, lock, accept)
        candidates = [Candidate(c.u_undistorted, c.v_undistorted, c.size_mm, c.width_ratio,
                                c.shape_ok) for c in msg.candidates]
        return View(candidates, geometry.camera_matrix(info.k), (info.width, info.height),
                    msg.status)

    def _settled(self, target: str, plane_distance_mm: float, lock: bool, accept):
        """First detection message after the request for which accept(msg). The request
        (target, distance) is sent only after settle_s, when the robot has rung down: it
        starts the tracks of vision_node over, so no image of the motion is in the median."""
        time.sleep(self.cfg["settle_s"])
        self._set_parameters(target, plane_distance_mm, lock)
        earliest = self.node.get_clock().now().nanoseconds
        deadline = time.monotonic() + self.cfg["timeout_s"]
        last_status = "no detection message"
        while time.monotonic() < deadline:
            with self._lock:
                msg, info = self._latest, self._info
            if msg is not None:
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if stamp >= earliest:
                    last_status = msg.status
                    if info is not None and accept(msg):
                        return msg, info
            time.sleep(POLL_S)
        raise RuntimeError(f"no {target} found within {self.cfg['timeout_s']} s "
                           f"(last status: {last_status})")
