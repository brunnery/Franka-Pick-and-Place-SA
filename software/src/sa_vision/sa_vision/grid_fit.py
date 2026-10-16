"""Accuracy of the nut centre on a printed grid of known positions. Pure NumPy, no ROS.

docs/THEORY.md, Section 2.8. The nut is placed on marks of known sheet coordinates
p_i (mm). For a camera looking straight down, the undistorted normalised image
coordinates n_i = ((u - c_x) / f_x, (v - c_y) / f_y) follow a similarity
n_i = (1/d) R p_i + t (rotation and offset of the sheet are unknown, the scale is
1/d). Fitting it by least squares gives d, and the residuals times d are the
position errors on the object plane in mm.
"""
from typing import Tuple

import numpy as np


def fit_similarity(src: np.ndarray, dst: np.ndarray) -> Tuple[float, np.ndarray, np.ndarray]:
    """Least-squares dst ~ scale * R @ src + t with a proper rotation R (Umeyama 1991).

    src, dst: (N, 2) with N >= 2. Returns (scale, R (2x2), t (2,)).
    """
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    if len(src) < 2 or src.shape != dst.shape:
        raise ValueError("need at least two point pairs of equal shape")
    mean_src, mean_dst = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - mean_src, dst - mean_dst
    u, sigma, vt = np.linalg.svd(b.T @ a / len(src))
    sign = np.diag([1.0, np.sign(np.linalg.det(u @ vt))])  # no reflection
    rotation = u @ sign @ vt
    variance = (a ** 2).sum() / len(src)
    scale = float(np.trace(np.diag(sigma) @ sign) / variance)
    return scale, rotation, mean_dst - scale * rotation @ mean_src


def similarity_residuals_mm(sheet_mm: np.ndarray, normalised: np.ndarray
                            ) -> Tuple[float, float, np.ndarray]:
    """Fit the similarity and return (d_mm, rotation_deg, residuals (N, 2) in mm).

    Residuals are measured minus fitted position, in camera axes (x right, y down
    in the image) on the object plane, i.e. normalised residual times d.
    """
    scale, rotation, offset = fit_similarity(sheet_mm, normalised)
    predicted = scale * np.asarray(sheet_mm, float) @ rotation.T + offset
    distance = 1.0 / scale
    angle = float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))
    return distance, angle, (np.asarray(normalised, float) - predicted) * distance


def homography_residuals_mm(sheet_mm: np.ndarray, normalised: np.ndarray,
                            distance_mm: float) -> np.ndarray:
    """Residuals (mm) of a full plane homography sheet -> image (N >= 4).

    The homography also absorbs a tilt of the sheet against the optical axis, so
    these residuals are the part of the error that no plane mapping can explain
    (detection noise and object-dependent bias).
    """
    src, dst = np.asarray(sheet_mm, float), np.asarray(normalised, float)
    if len(src) < 4:
        raise ValueError("a homography needs at least four points")
    rows = []
    for (x, y), (u, v) in zip(src, dst):
        rows.append([x, y, 1, 0, 0, 0, -u * x, -u * y, -u])
        rows.append([0, 0, 0, x, y, 1, -v * x, -v * y, -v])
    h = np.linalg.svd(np.asarray(rows))[2][-1].reshape(3, 3)
    mapped = np.c_[src, np.ones(len(src))] @ h.T
    predicted = mapped[:, :2] / mapped[:, 2:]
    return (dst - predicted) * distance_mm
