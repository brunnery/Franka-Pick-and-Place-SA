"""Read tasks.yaml and read/write poses.yaml. No ROS.

Missing keys raise KeyError on purpose (no hidden defaults, CLAUDE.md).
"""
from pathlib import Path

import numpy as np
import yaml

from sa_tasks import frames
from sa_tasks.trajectory import MotionLimits

PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_TASKS = PACKAGE_DIR / "config" / "tasks.yaml"
DEFAULT_POSES = PACKAGE_DIR / "config" / "poses.yaml"
POSES_HEADER = """\
# Taught poses, written by `ros2 run sa_tasks teach_poses`: flange pose in the base frame,
# position in mm, quaternion (x, y, z, w). Only the position is used; the orientation is
# always the fixed one of tasks.yaml.
"""


def load_tasks(path: Path = DEFAULT_TASKS) -> dict:
    """tasks.yaml (symlinks of --symlink-install resolved). Refuses CAD offsets that do not
    close: flange -> camera + camera -> TCP must equal flange -> TCP (THEORY 0)."""
    cfg = yaml.safe_load(Path(path).resolve().read_text())
    f = cfg["frames"]
    error = (np.add(f["flange_to_camera_housing_mm"], f["camera_housing_to_tcp_mm"])
             - f["flange_to_tcp_mm"])
    if np.any(np.abs(error) > f["offset_chain_tolerance_mm"]):
        raise ValueError(f"frames: CAD offsets do not close, off by "
                         f"{np.round(error, 3).tolist()} mm")
    return cfg


def motion_limits(cfg: dict) -> MotionLimits:
    m = cfg["robot"]["motion"]
    return MotionLimits(m["max_speed_mm_s"], m["max_angular_speed_deg_s"], m["lead_fraction"],
                        m["min_duration_s"])


def rotation_down(cfg: dict) -> np.ndarray:
    """R_B_F of the only commanded orientation (quaternion in tasks.yaml)."""
    return frames.quaternion_to_matrix(cfg["frames"]["flange_orientation_xyzw"])


def link8_to_flange(cfg: dict) -> np.ndarray:
    return frames.rotation_z(cfg["frames"]["link8_to_flange_yaw_deg"])


def camera_in_flange(cfg: dict) -> np.ndarray:
    """T_F_C: the projection centre lies above the lowest point of the housing."""
    f = cfg["frames"]
    return frames.pose(f["camera_rotation_in_flange"], np.subtract(
        f["flange_to_camera_housing_mm"], [0.0, 0.0, f["projection_centre_above_housing_mm"]]))


class Poses:
    """Taught flange poses and the table height (poses.yaml)."""

    def __init__(self, path: Path = DEFAULT_POSES):
        self.path = Path(path).resolve()
        data = yaml.safe_load(self.path.read_text()) or {}
        self.table_z_mm = data.get("table_z_mm")
        self.poses = dict(data.get("poses") or {})

    def table_z(self) -> float:
        if self.table_z_mm is None:
            raise RuntimeError("table height not taught: run teach_poses and use 'table'")
        return float(self.table_z_mm)

    def get(self, name: str) -> np.ndarray:
        entry = self.poses.get(name)
        if entry is None:
            raise RuntimeError(f"pose '{name}' not taught: run teach_poses and use 'save {name}'")
        return frames.pose(frames.quaternion_to_matrix(entry["quaternion_xyzw"]),
                           entry["position_mm"])

    def set(self, name: str, t_base_flange: np.ndarray) -> None:
        self.poses[name] = {
            "position_mm": [round(float(v), 2) for v in t_base_flange[:3, 3]],
            "quaternion_xyzw": [round(float(v), 6)
                                for v in frames.matrix_to_quaternion(t_base_flange[:3, :3])]}

    def save(self) -> None:
        data = {"table_z_mm": None if self.table_z_mm is None else round(self.table_z_mm, 2),
                "poses": self.poses}
        self.path.write_text(POSES_HEADER + yaml.safe_dump(data, sort_keys=False))
