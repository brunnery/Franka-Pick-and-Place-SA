"""Rigid transforms between base, flange, end effector, camera and TCP. Pure NumPy, no ROS.

docs/THEORY.md, Sections 0 and 3. Lengths in mm, poses as 4x4 matrices T_a_b (pose of b
in a). Frames: B base, F flange (centre of the contact face, axes of the Franka hand =
fr3_link8 turned -45 deg about z), L fr3_link8, E libfranka end effector (Desk),
C camera projection centre (OpenCV axes), T TCP = centre of the gripper bottom plate.
"""
from typing import Sequence

import numpy as np


def pose(rotation, translation) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = np.asarray(rotation, float)
    t[:3, 3] = np.asarray(translation, float)
    return t


def invert(t: np.ndarray) -> np.ndarray:
    r, p = t[:3, :3], t[:3, 3]
    return pose(r.T, -r.T @ p)


def quaternion_to_matrix(xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = np.asarray(xyzw, float) / np.linalg.norm(xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(r: np.ndarray) -> np.ndarray:
    """Unit quaternion (x, y, z, w) with w >= 0 (Shepperd's method)."""
    r = np.asarray(r, float)
    trace = np.trace(r)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0] * 4
        q[i] = s / 4
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
        q[3] = (r[k, j] - r[j, k]) / s
    q = np.asarray(q)
    return -q if q[3] < 0 else q


def slerp(q0, q1, s: float) -> np.ndarray:
    """Spherical interpolation of unit quaternions (x, y, z, w), shortest path."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    dot = float(q0 @ q1)
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:  # nearly equal: linear interpolation is exact enough
        q = q0 + s * (q1 - q0)
        return q / np.linalg.norm(q)
    angle = np.arccos(dot)
    return (np.sin((1 - s) * angle) * q0 + np.sin(s * angle) * q1) / np.sin(angle)


def rotation_angle_deg(r0: np.ndarray, r1: np.ndarray) -> float:
    """Angle of the rotation that takes r0 to r1."""
    return float(np.degrees(np.arccos(np.clip((np.trace(r0.T @ r1) - 1.0) / 2.0, -1.0, 1.0))))


def tilt_deg(r_base_flange: np.ndarray) -> float:
    """Angle between the tool axis z_F and the downward vertical."""
    return float(np.degrees(np.arccos(np.clip(-r_base_flange[2, 2], -1.0, 1.0))))


def rotation_z(angle_deg: float) -> np.ndarray:
    a = np.radians(angle_deg)
    return np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])


def rotation_xy(ax_deg: float, ay_deg: float) -> np.ndarray:
    """Rotation about x by ax_deg, then about the new y by ay_deg (small tilts)."""
    a, b = np.radians(ax_deg), np.radians(ay_deg)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(a), -np.sin(a)], [0.0, np.sin(a), np.cos(a)]])
    ry = np.array([[np.cos(b), 0.0, np.sin(b)], [0.0, 1.0, 0.0], [-np.sin(b), 0.0, np.cos(b)]])
    return rx @ ry


def flange_from_state(t_base_ee: np.ndarray, t_link8_ee: np.ndarray,
                      r_link8_flange: np.ndarray) -> tuple:
    """(T_B_F, T_F_E) from the robot state (O_T_EE, F_T_EE of libfranka, whose "F" is
    fr3_link8): T_B_F = T_B_E (T_L_E)^-1 T_L_F, T_L_F a pure rotation (same origin)."""
    t_link8_flange = pose(r_link8_flange, [0.0, 0.0, 0.0])
    return t_base_ee @ invert(t_link8_ee) @ t_link8_flange, invert(t_link8_flange) @ t_link8_ee


def flange_for_tcp(tcp_mm, r_base_flange: np.ndarray, flange_to_tcp_mm) -> np.ndarray:
    """Flange pose that puts the TCP at tcp_mm with the given flange orientation."""
    r = np.asarray(r_base_flange, float)
    return pose(r, np.asarray(tcp_mm, float) - r @ np.asarray(flange_to_tcp_mm, float))


def tcp_position(t_base_flange: np.ndarray, flange_to_tcp_mm) -> np.ndarray:
    return (t_base_flange @ np.append(np.asarray(flange_to_tcp_mm, float), 1.0))[:3]


def object_in_base(u: float, v: float, k: np.ndarray, t_base_camera: np.ndarray,
                   plane_z_mm: float) -> np.ndarray:
    """Intersection of the viewing ray of an undistorted pixel with the plane z_B = plane_z
    (THEORY 3.1): r_B = R_BC K^-1 (u, v, 1), p = o_C + lambda r_B."""
    ray = t_base_camera[:3, :3] @ np.linalg.solve(k, [u, v, 1.0])
    origin = t_base_camera[:3, 3]
    if ray[2] >= 0.0 or origin[2] <= plane_z_mm:
        raise ValueError("viewing ray does not hit the object plane")
    return origin + (plane_z_mm - origin[2]) / ray[2] * ray


def distance_along_axis(t_base_camera: np.ndarray, plane_z_mm: float) -> float:
    """Distance from the projection centre to the plane z_B = plane_z along z_C."""
    return float((t_base_camera[2, 3] - plane_z_mm) / max(-t_base_camera[2, 2], 1e-6))
