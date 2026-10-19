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
    cfg = yaml.safe_load(resolve(path).read_text())
    check_offset_chain(cfg)
    return cfg


def offset_chain_error(cfg: dict) -> np.ndarray:
    """(flange -> camera housing) + (camera housing -> TCP) - (flange -> TCP), in {F}, mm.
    Two independent CAD measurements of the same mount; they must close (THEORY 0)."""
    f = cfg["frames"]
    return (np.asarray(f["flange_to_camera_housing_mm"], float)
            + np.asarray(f["camera_housing_to_tcp_mm"], float)
            - np.asarray(f["flange_to_tcp_mm"], float))


def check_offset_chain(cfg: dict) -> None:
    """Refuse a config whose camera and TCP offsets contradict each other (e.g. one of them
    updated after a CAD change and the other not)."""
    error = offset_chain_error(cfg)
    tolerance = cfg["frames"]["offset_chain_tolerance_mm"]
    if np.any(np.abs(error) > tolerance):
        raise ValueError("frames: flange_to_camera_housing_mm + camera_housing_to_tcp_mm != "
                         f"flange_to_tcp_mm, off by {np.round(error, 3).tolist()} mm "
                         f"(tolerance {tolerance} mm per axis): check the CAD offsets")


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


def tool_in_flange(cfg: dict, poses: "Poses") -> np.ndarray:
    """T_F_G: the gripper ("tool") frame G is the flange frame tilted by the measured mount
    tilt (poses.yaml, teach_poses 'flat'); same origin. Identity if use_gripper_tilt is
    false. The CAD offsets (camera, TCP) are defined in G."""
    if not cfg["frames"]["use_gripper_tilt"]:
        return np.eye(4)
    if poses.gripper_tilt_deg is None:
        raise RuntimeError("use_gripper_tilt is on, but no tilt measured: teach_poses 'flat'")
    return frames.pose(frames.matrix_from_rotation_vector_deg(poses.gripper_tilt_deg),
                       [0.0, 0.0, 0.0])


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
        # Mount tilt of the gripper against the flange: rotation vector (deg) of T_F_G,
        # the mean of the samples taken with the bottom plate pressed flat on the table
        self.gripper_tilt_deg: Optional[list] = data.get("gripper_tilt_deg")
        self.gripper_tilt_samples: list = list(data.get("gripper_tilt_samples") or [])
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

    def add_tilt_sample(self, t_base_flange: np.ndarray) -> np.ndarray:
        """Bottom plate pressed flat on the table: the gripper frame is level (same yaw), so
        R_F_G = R_B_F^T level(R_B_F). Stores the sample and the mean; returns the sample."""
        r = t_base_flange[:3, :3]
        sample = frames.rotation_vector_deg(r.T @ frames.level(r))
        self.gripper_tilt_samples.append([round(float(v), 3) for v in sample])
        mean = np.mean(np.array(self.gripper_tilt_samples), axis=0)
        self.gripper_tilt_deg = [round(float(v), 3) for v in mean]
        return sample

    def save(self) -> None:
        data = {"table_z_mm": None if self.table_z_mm is None else round(self.table_z_mm, 2),
                "gripper_tilt_deg": self.gripper_tilt_deg,
                "gripper_tilt_samples": self.gripper_tilt_samples,
                "poses": self.poses}
        self.path.write_text(POSES_HEADER + yaml.safe_dump(data, sort_keys=False))
