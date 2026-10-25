import numpy as np
import pytest

from sa_tasks import frames
from sa_tasks.hand_eye_solver import camera_rotation, xyz_angles_deg

R_CAD = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
R_DOWN = frames.quaternion_to_matrix([1.0, 0.0, 0.0, 0.0])
T_FC = np.array([44.4, 0.0, 48.2])
BOARD = np.array([460.0, -200.0, 12.0])


def synthetic(r_fc, flange_tilt_deg=0.3, noise_mm=0.0, seed=0):
    rng = np.random.default_rng(seed)
    rs, ps, os_ = [], [], []
    for dx in (-40.0, 0.0, 40.0):
        for dy in (-50.0, 0.0, 50.0):
            for dz in (-80.0, 0.0, 80.0):
                r = frames.rotation_xy(*rng.normal(0.0, flange_tilt_deg, 2)) @ R_DOWN
                p = np.array([420.0 + dx, -200.0 + dy, 420.0 + dz])
                o = r_fc.T @ (r.T @ (BOARD - p) - T_FC) + rng.normal(0.0, noise_mm, 3)
                rs.append(r), ps.append(p), os_.append(o)
    return rs, ps, os_


def test_recovers_a_tilted_and_turned_camera():
    true = frames.rotation_xy(1.2, -0.7) @ frames.rotation_z(0.5) @ R_CAD
    r_fc, board, residuals = camera_rotation(*synthetic(true), T_FC, R_CAD, 50)
    assert xyz_angles_deg(r_fc @ R_CAD.T) == pytest.approx([1.2, -0.7, 0.5], abs=1e-3)
    assert board == pytest.approx(BOARD, abs=1e-3)
    assert residuals.max() < 1e-3


def test_noise_of_half_a_millimetre_costs_little_angle():
    true = frames.rotation_xy(1.0, 0.5) @ R_CAD
    r_fc, _, residuals = camera_rotation(*synthetic(true, noise_mm=0.5), T_FC, R_CAD, 50)
    assert xyz_angles_deg(r_fc @ R_CAD.T)[:2] == pytest.approx([1.0, 0.5], abs=0.15)
    assert residuals.mean() < 1.5
