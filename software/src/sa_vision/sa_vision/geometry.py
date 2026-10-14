"""Pixel -> 3D point by back-projection. Pure NumPy/OpenCV, no ROS.

docs/THEORY.md, Section 1.2 (undistortion) and Section 3 (ray-plane intersection).
Lengths are unit-agnostic: the result has the unit of the distances passed in.
"""
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


def camera_matrix(k: Sequence[float]) -> np.ndarray:
    """3x3 intrinsic matrix K from the row-major list of sensor_msgs/CameraInfo.k."""
    return np.asarray(k, dtype=float).reshape(3, 3)


def undistort_pixel(u: float, v: float, k: np.ndarray, dist: Sequence[float]) -> Tuple[float, float]:
    """Remove lens distortion from one pixel (iterative inverse of Brown-Conrady).

    Returns the pixel the same point would have in an ideal pinhole camera with
    the same K. Empty or all-zero dist leaves the pixel unchanged.
    """
    dist = np.asarray(dist, dtype=float)
    if dist.size == 0 or not np.any(dist):
        return float(u), float(v)
    points = np.array([[[u, v]]], dtype=float)
    undistorted = cv2.undistortPoints(points, k, dist, P=k)
    return float(undistorted[0, 0, 0]), float(undistorted[0, 0, 1])


def pixel_ray(u: float, v: float, k: np.ndarray) -> np.ndarray:
    """Viewing ray r_C = K^-1 (u, v, 1)^T of an undistorted pixel (z component 1)."""
    return np.linalg.solve(k, np.array([u, v, 1.0]))


def pixel_to_camera_point(u: float, v: float, k: np.ndarray, distance: float) -> np.ndarray:
    """Point on the plane z_C = d hit by the ray of an undistorted pixel.

    Special case for a camera looking straight at the plane:
    X_C = (u - c_x) d / f_x, Y_C = (v - c_y) d / f_y, Z_C = d.
    """
    return pixel_ray(u, v, k) * distance


def intersect_ray_plane(origin: np.ndarray, direction: np.ndarray, plane_point: np.ndarray,
                        plane_normal: np.ndarray, eps: float = 1e-12) -> Optional[np.ndarray]:
    """Intersection of p(l) = o + l * r with the plane n . (p - p0) = 0.

    Returns None if the ray is parallel to the plane or the plane lies behind
    the origin (l <= 0).
    """
    origin, direction = np.asarray(origin, float), np.asarray(direction, float)
    plane_point, plane_normal = np.asarray(plane_point, float), np.asarray(plane_normal, float)
    denom = plane_normal @ direction
    if abs(denom) < eps:
        return None
    lam = plane_normal @ (plane_point - origin) / denom
    if lam <= 0.0:
        return None
    return origin + lam * direction


def pixel_to_plane_point(u: float, v: float, k: np.ndarray, t_ref_cam: np.ndarray,
                         plane_height: float) -> Optional[np.ndarray]:
    """Back-project an undistorted pixel onto the horizontal plane z = plane_height.

    t_ref_cam: 4x4 pose of the camera in the reference frame (e.g. base frame,
    T_base_cam = T_base_flange * T_flange_cam). The plane is horizontal in that frame.
    """
    t_ref_cam = np.asarray(t_ref_cam, float)
    rotation, origin = t_ref_cam[:3, :3], t_ref_cam[:3, 3]
    direction = rotation @ pixel_ray(u, v, k)
    return intersect_ray_plane(origin, direction, np.array([0.0, 0.0, plane_height]),
                               np.array([0.0, 0.0, 1.0]))
