"""Rotation of the camera in the flange from pure translations of the flange. No ROS.

docs/THEORY.md 1.5. Board fixed on the table, seen from n flange poses (R_i, p_i) in the base
frame with the board origin at o_i in the camera frame (solvePnP). With the camera at t (CAD)
in the flange and R_FC unknown, every pose must give the same board origin b in the base:
    p_i + R_i (t + R_FC o_i) = b.
Gauss-Newton on (R_FC, b): with R_FC = exp([d]x) R_k and v_i = R_k o_i the equations are linear
in the small rotation d and in b: R_i [v_i]x d + b = p_i + R_i (t + v_i). With an (almost)
constant flange orientation t and b cannot be told apart, so t is taken from the CAD; the
rotation follows from how the board moves in the image against the flange.
"""
import numpy as np


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rotation_from_vector(d: np.ndarray) -> np.ndarray:
    """exp([d]x) (Rodrigues)."""
    angle = float(np.linalg.norm(d))
    if angle < 1e-12:
        return np.eye(3)
    k = skew(d / angle)
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * k @ k


def camera_rotation(r_base_flange, p_base_flange, p_camera_board, t_flange_camera,
                    r_start: np.ndarray, iterations: int) -> tuple:
    """(R_FC, board origin in the base, residual per pose in mm). r_start: e.g. the CAD.
    Stops early once the rotation step is below 1e-10 rad."""
    rs = np.asarray(r_base_flange, float)
    ps = np.asarray(p_base_flange, float)
    os_ = np.asarray(p_camera_board, float)
    t = np.asarray(t_flange_camera, float)
    r_fc = np.asarray(r_start, float)
    for _ in range(iterations):
        rows, rhs = [], []
        for r, p, o in zip(rs, ps, os_):
            v = r_fc @ o
            rows.append(np.hstack([r @ skew(v), np.eye(3)]))
            rhs.append(p + r @ (t + v))
        solution = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)[0]
        r_fc = rotation_from_vector(solution[:3]) @ r_fc
        if np.linalg.norm(solution[:3]) < 1e-10:
            break
    board = np.mean([p + r @ (t + r_fc @ o) for r, p, o in zip(rs, ps, os_)], axis=0)
    residuals = np.array([np.linalg.norm(p + r @ (t + r_fc @ o) - board)
                          for r, p, o in zip(rs, ps, os_)])
    return r_fc, board, residuals


def xyz_angles_deg(r: np.ndarray) -> np.ndarray:
    """Angles (a, b, c) with r = R_x(a) R_y(b) R_z(c): a, b as frames.rotation_xy, then z."""
    b = np.arcsin(np.clip(r[0, 2], -1.0, 1.0))
    a = np.arctan2(-r[1, 2], r[2, 2])
    c = np.arctan2(-r[0, 1], r[0, 0])
    return np.degrees([a, b, c])
