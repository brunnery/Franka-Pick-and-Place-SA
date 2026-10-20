"""Hardware interfaces used by the sequences, and the mocks of `--robot mock`. No ROS.

The real robot and vision live in ros_interfaces.py; the sequences only see these methods.
"""
import logging
from dataclasses import dataclass
from typing import List, Optional, Protocol

import numpy as np

from sa_tasks.trajectory import Waypoint

log = logging.getLogger("sa_tasks")


@dataclass
class Observation:
    """A settled detection: undistorted pixel of the object centre and the intrinsics."""
    u_undistorted: float
    v_undistorted: float
    k: np.ndarray


@dataclass
class Candidate:
    """One object in view (Detection.candidates), ranked by vision_node."""
    u_undistorted: float
    v_undistorted: float
    size_mm: float
    width_ratio: float
    shape_ok: bool


@dataclass
class View:
    """The objects of one settled view, most likely target first, and the intrinsics."""
    candidates: List[Candidate]
    k: np.ndarray


class Robot(Protocol):
    def flange_pose(self) -> np.ndarray: ...                       # T_B_F, mm
    def force_z(self) -> float: ...                                # external force along z_B, N
    def set_mode(self, mode: int) -> None: ...
    def follow(self, path: List[Waypoint], stop_on_contact_n: Optional[float] = None) -> bool: ...
    def wait_until_arrived(self, goal: np.ndarray, tolerance_mm: float) -> None: ...


class Gripper(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def rotate(self, direction: int, duration_ms: int, speed: int) -> None: ...
    def shutdown(self) -> None: ...


class Vision(Protocol):
    def prepare(self, target: str) -> None: ...                   # select the target early
    def observe(self, target: str, plane_distance_mm: float) -> Observation: ...
    # lock: choose among objects confirmed over several images (vision_node locks on them);
    # else the median over the last images (measuring a centred object)
    def view(self, target: str, plane_distance_mm: float, lock: bool) -> View: ...


class MockRobot:
    """Jumps to every waypoint; a surface at contact_tcp_z_mm (TCP z) produces a force."""

    def __init__(self, start: np.ndarray, flange_to_tcp_z_mm: float,
                 contact_tcp_z_mm: Optional[float] = None, stiffness_n_mm: float = 1.5):
        self.pose = start.copy()
        self.mode = 0
        self.tcp_z_offset = flange_to_tcp_z_mm
        self.contact_tcp_z = contact_tcp_z_mm
        self.stiffness = stiffness_n_mm
        self.history: List[np.ndarray] = []

    def flange_pose(self) -> np.ndarray:
        return self.pose.copy()

    def force_z(self) -> float:
        if self.contact_tcp_z is None:
            return 0.0
        return self.stiffness * max(self.contact_tcp_z - (self.pose[2, 3] - self.tcp_z_offset), 0.0)

    def set_mode(self, mode: int) -> None:
        self.mode = mode

    def follow(self, path: List[Waypoint], stop_on_contact_n: Optional[float] = None) -> bool:
        baseline = self.force_z()
        for waypoint in path:
            self.pose = waypoint.t_base_flange.copy()
            self.history.append(self.pose)
            if stop_on_contact_n is not None and abs(self.force_z() - baseline) > stop_on_contact_n:
                return True
        return False

    def wait_until_arrived(self, goal: np.ndarray, tolerance_mm: float) -> None:
        error = float(np.linalg.norm(self.pose[:3, 3] - goal[:3, 3]))
        if error > tolerance_mm:
            raise RuntimeError(f"[mock robot] not at goal ({error:.1f} mm)")


class MockGripper:
    """Logs the commands (the real gripper waits for DONE)."""

    def __init__(self):
        self.commands: List[str] = []

    def open(self) -> None:
        self._log("OPEN")

    def close(self) -> None:
        self._log("CLOSE")

    def rotate(self, direction: int, duration_ms: int, speed: int) -> None:
        self._log("ROT")

    def shutdown(self) -> None:
        pass

    def _log(self, command: str) -> None:
        log.info("[mock gripper] %s", command)
        self.commands.append(command)
