"""Rigid transforms between base, flange, end effector, camera and TCP. Pure NumPy, no ROS.

docs/THEORY.md, Sections 0 and 3. All lengths in mm, poses as 4x4 homogeneous
matrices T_a_b (pose of frame b in frame a). Frames:
  B  robot base
  F  flange: origin at the centre of the flange contact face, axes of the Franka hand
     / EE frame = fr3_link8 turned by -45 deg about z (CLAUDE.md, THEORY 0)
  L  fr3_link8 (libfranka's "flange" frame; the robot state gives L_T_EE)
  E  libfranka end effector   (set in Desk)
  C  camera projection centre (OpenCV axes)
  T  TCP = centre of the gripper bottom plate
"""
from typing import Sequence, Tuple

import numpy as np

from sa_vision import geometry


def pose(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
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


def slerp(q0: Sequence[float], q1: Sequence[float], s: float) -> np.ndarray:
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
    c = (np.trace(r0.T @ r1) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def level(r_base_frame: np.ndarray) -> np.ndarray:
    """Same yaw, z axis exactly down: x projected onto the horizontal plane, z = -z_B."""
    x = np.array([r_base_frame[0, 0], r_base_frame[1, 0], 0.0])
    x /= np.linalg.norm(x)
    z = np.array([0.0, 0.0, -1.0])
    return np.stack([x, np.cross(z, x), z], axis=1)


def rotation_vector_deg(r: np.ndarray) -> np.ndarray:
    """Axis * angle (deg) of a rotation matrix (log map, angles below 180 deg)."""
    angle = np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * np.sin(angle))
    return np.degrees(angle) * axis


def matrix_from_rotation_vector_deg(v) -> np.ndarray:
    """Rodrigues: rotation matrix of axis * angle (deg)."""
    v = np.radians(np.asarray(v, float))
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def tilt_deg(r_base_flange: np.ndarray) -> float:
    """Angle between the tool axis z_F and the downward vertical."""
    return float(np.degrees(np.arccos(np.clip(-r_base_flange[2, 2], -1.0, 1.0))))


def rotation_z(angle_deg: float) -> np.ndarray:
    a = np.radians(angle_deg)
    return np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])


def flange_to_camera(r_flange_camera: np.ndarray, flange_to_housing_mm: Sequence[float],
                     projection_centre_above_housing_mm: float) -> np.ndarray:
    """T_F_C: the projection centre lies above the lowest point of the housing (towards F)."""
    t = np.asarray(flange_to_housing_mm, float) - [0.0, 0.0, projection_centre_above_housing_mm]
    return pose(r_flange_camera, t)


def flange_from_state(t_base_ee: np.ndarray, t_link8_ee: np.ndarray,
                      r_link8_flange: np.ndarray) -> tuple:
    """(T_B_F, T_F_E) from the robot state (O_T_EE, F_T_EE of libfranka, whose "F" is
    fr3_link8). T_B_F = T_B_E (T_L_E)^-1 T_L_F with T_L_F a pure rotation (same origin)."""
    t_link8_flange = pose(r_link8_flange, [0.0, 0.0, 0.0])
    t_base_flange = t_base_ee @ invert(t_link8_ee) @ t_link8_flange
    return t_base_flange, invert(t_link8_flange) @ t_link8_ee


def flange_from_end_effector(t_base_ee: np.ndarray, t_flange_ee: np.ndarray) -> np.ndarray:
    """T_B_F = T_B_E * (T_F_E)^-1."""
    return t_base_ee @ invert(t_flange_ee)


def end_effector_from_flange(t_base_flange: np.ndarray, t_flange_ee: np.ndarray) -> np.ndarray:
    """T_B_E = T_B_F * T_F_E: the pose the controller has to be commanded."""
    return t_base_flange @ t_flange_ee


def flange_for_tcp(tcp_mm: Sequence[float], r_base_flange: np.ndarray,
                   flange_to_tcp_mm: Sequence[float]) -> np.ndarray:
    """Flange pose that puts the TCP at tcp_mm with the given flange orientation."""
    r = np.asarray(r_base_flange, float)
    return pose(r, np.asarray(tcp_mm, float) - r @ np.asarray(flange_to_tcp_mm, float))


def tcp_position(t_base_flange: np.ndarray, flange_to_tcp_mm: Sequence[float]) -> np.ndarray:
    return (t_base_flange @ np.append(np.asarray(flange_to_tcp_mm, float), 1.0))[:3]


def object_in_base(u_undistorted: float, v_undistorted: float, k: np.ndarray,
                   t_base_camera: np.ndarray, plane_z_mm: float) -> np.ndarray:
    """Intersection of the viewing ray of an undistorted pixel with the plane z_B = plane_z.

    THEORY 3.1: r_C = K^-1 (u, v, 1), r_B = R_BC r_C, p = o_C + lambda r_B on the plane.
    Raises ValueError if the ray does not hit the plane in front of the camera.
    """
    point = geometry.pixel_to_plane_point(u_undistorted, v_undistorted, k, t_base_camera,
                                          plane_z_mm)
    if point is None:
        raise ValueError("viewing ray does not hit the object plane")
    return point


def distance_along_axis(t_base_camera: np.ndarray, plane_z_mm: float) -> float:
    """Distance from the projection centre to the plane z_B = plane_z along z_C (camera
    looking down: the height difference divided by the cosine of the tilt)."""
    z_axis = t_base_camera[:3, 2]
    height = t_base_camera[2, 3] - plane_z_mm
    return float(height / max(-z_axis[2], 1e-6))


def footprint_mm(distance_mm: float, k: np.ndarray, image_size: Tuple[int, int]) -> Tuple[float, float]:
    """Width and height of the image on a plane at distance d (pinhole)."""
    width, height = image_size
    return width * distance_mm / k[0, 0], height * distance_mm / k[1, 1]
