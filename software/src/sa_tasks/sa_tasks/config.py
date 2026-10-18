"""Read tasks.yaml and read/write poses.yaml. No ROS.

Missing keys raise KeyError on purpose (no hidden defaults, CLAUDE.md).
"""
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import yaml

from sa_tasks import frames
from sa_tasks.trajectory import MotionLimits

PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_TASKS = PACKAGE_DIR / "config" / "tasks.yaml"
DEFAULT_POSES = PACKAGE_DIR / "config" / "poses.yaml"
POSES_HEADER = """\
# Taught poses, written by `ros2 run sa_tasks teach_poses`. Do not edit by hand unless you
# know the values: flange pose in the base frame, position in mm, quaternion (x, y, z, w).
# Observation poses are commanded with the fixed downward orientation of tasks.yaml;
# the taught orientation is only checked (tilt).
"""


def resolve(path: Path) -> Path:
    """Follow the symlinks of a --symlink-install, so the source file is read and written."""
    return Path(path).resolve()


def load_tasks(path: Path = DEFAULT_TASKS) -> dict:
    return yaml.safe_load(resolve(path).read_text())


def motion_limits(cfg: dict) -> MotionLimits:
    m = cfg["robot"]["motion"]
    return MotionLimits(m["max_speed_mm_s"], m["max_angular_speed_deg_s"], m["lead_fraction"],
                        m["min_duration_s"])


def _rotation(rows, name: str) -> np.ndarray:
    r = np.asarray(rows, float)
    if not np.allclose(r @ r.T, np.eye(3), atol=1e-6) or np.linalg.det(r) < 0:
        raise ValueError(f"{name} is not a rotation matrix")
    return r


def rotation_down(cfg: dict) -> np.ndarray:
    """R_B_F of the only commanded orientation (quaternion in tasks.yaml)."""
    return frames.quaternion_to_matrix(cfg["frames"]["flange_orientation_xyzw"])


def link8_to_flange(cfg: dict) -> np.ndarray:
    return frames.rotation_z(cfg["frames"]["link8_to_flange_yaw_deg"])


def camera_in_flange(cfg: dict) -> np.ndarray:
    f = cfg["frames"]
    return frames.flange_to_camera(_rotation(f["camera_rotation_in_flange"],
                                             "frames.camera_rotation_in_flange"),
                                   f["flange_to_camera_housing_mm"],
                                   f["projection_centre_above_housing_mm"])


class Poses:
    """Taught flange poses and the table height (poses.yaml)."""

    def __init__(self, path: Path = DEFAULT_POSES):
        self.path = resolve(path)
        data = yaml.safe_load(self.path.read_text()) or {}
        self.table_z_mm: Optional[float] = data.get("table_z_mm")
        self.poses: Dict[str, Optional[dict]] = dict(data.get("poses") or {})

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
                                for v in frames.matrix_to_quaternion(t_base_flange[:3, :3])],
        }

    def save(self) -> None:
        data = {"table_z_mm": None if self.table_z_mm is None else round(self.table_z_mm, 2),
                "poses": self.poses}
        self.path.write_text(POSES_HEADER + yaml.safe_dump(data, sort_keys=False))
