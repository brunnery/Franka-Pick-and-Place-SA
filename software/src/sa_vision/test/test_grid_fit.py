import numpy as np
import pytest

from sa_vision.grid_fit import fit_similarity, homography_residuals_mm, similarity_residuals_mm

GRID = np.array([[x, y] for y in (-45.0, -15.0, 15.0, 45.0)
                 for x in (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0)])


def project(points, distance, angle_deg, offset):
    a = np.radians(angle_deg)
    rotation = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    return points @ rotation.T / distance + offset


def test_fit_similarity_recovers_scale_rotation_offset():
    dst = project(GRID, 400.0, 3.0, np.array([0.01, -0.02]))
    scale, rotation, offset = fit_similarity(GRID, dst)
    assert scale == pytest.approx(1 / 400.0)
    assert np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])) == pytest.approx(3.0)
    assert offset == pytest.approx([0.01, -0.02])


def test_fit_similarity_never_returns_a_reflection():
    mirrored = GRID * np.array([-1.0, 1.0]) / 400.0
    _, rotation, _ = fit_similarity(GRID, mirrored)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_similarity_residuals_are_the_injected_errors_in_mm():
    errors_mm = np.zeros_like(GRID)
    errors_mm[5] = [1.0, -0.5]
    errors_mm[20] = [-1.0, 0.5]  # zero mean, so the fit is not shifted
    normalised = project(GRID, 400.0, -2.0, np.zeros(2)) + errors_mm / 400.0
    distance, angle, residuals = similarity_residuals_mm(GRID, normalised)
    assert distance == pytest.approx(400.0, rel=1e-3)
    assert angle == pytest.approx(-2.0, abs=0.05)
    # Least squares spreads a little of each outlier over the fit (small rotation)
    assert residuals[5] == pytest.approx([1.0, -0.5], abs=0.1)
    assert np.abs(np.delete(residuals, [5, 20], axis=0)).max() < 0.1


def test_homography_absorbs_a_tilted_sheet():
    # Sheet tilted by 2 deg about the image x axis: depth varies with y
    tilt = np.radians(2.0)
    depth = 400.0 + GRID[:, 1] * np.sin(tilt)
    normalised = np.c_[GRID[:, 0], GRID[:, 1] * np.cos(tilt)] / depth[:, None]
    _, _, similarity = similarity_residuals_mm(GRID, normalised)
    homography = homography_residuals_mm(GRID, normalised, 400.0)
    assert np.abs(similarity).max() > 0.1
    assert np.abs(homography).max() < 1e-6
