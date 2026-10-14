import cv2
import numpy as np
import pytest

from sa_vision.geometry import (camera_matrix, intersect_ray_plane, pixel_ray,
                                pixel_to_camera_point, pixel_to_plane_point, undistort_pixel)

# Calibration of 14.10.2026 (calibration/hd_camera.yaml)
K = camera_matrix([2077.155838, 0.0, 695.441022, 0.0, 2083.725681, 405.574535, 0.0, 0.0, 1.0])
DIST = [0.137350, -0.749399, -0.002155, 0.001190, 0.0]
CX, CY, FX, FY = K[0, 2], K[1, 2], K[0, 0], K[1, 1]


def rot_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def pose(rotation, translation):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rotation, translation
    return t


def test_camera_matrix_from_camera_info_list():
    assert K.shape == (3, 3)
    assert (FX, CX, FY, CY) == pytest.approx((2077.155838, 695.441022, 2083.725681, 405.574535))


def test_undistort_without_distortion_is_identity():
    assert undistort_pixel(100.0, 200.0, K, []) == (100.0, 200.0)
    assert undistort_pixel(100.0, 200.0, K, [0, 0, 0, 0, 0]) == (100.0, 200.0)


def test_principal_point_is_not_moved_by_distortion():
    assert undistort_pixel(CX, CY, K, DIST) == pytest.approx((CX, CY), abs=1e-6)


@pytest.mark.parametrize("u, v, corner", [(100.0, 100.0, True), (1200.0, 650.0, True),
                                          (640.0, 360.0, False)])
def test_undistort_inverts_projection(u, v, corner):
    # Ideal pixel -> distorted pixel with OpenCV's forward model -> back.
    ray = pixel_ray(u, v, K)
    distorted, _ = cv2.projectPoints(ray.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K,
                                     np.array(DIST))
    ud, vd = distorted[0, 0]
    if corner:
        assert (ud, vd) != pytest.approx((u, v), abs=0.5)  # distortion is noticeable here
    assert undistort_pixel(ud, vd, K, DIST) == pytest.approx((u, v), abs=0.01)


def test_camera_point_pinhole_relation():
    d = 300.0
    u, v = CX + 207.7, CY - 104.2
    point = pixel_to_camera_point(u, v, K, d)
    assert point == pytest.approx([(u - CX) * d / FX, (v - CY) * d / FY, d])
    assert pixel_to_camera_point(CX, CY, K, d) == pytest.approx([0.0, 0.0, d])


def test_ray_plane_intersection():
    hit = intersect_ray_plane(np.array([0, 0, 10.0]), np.array([1.0, 0, -1.0]),
                              np.zeros(3), np.array([0, 0, 1.0]))
    assert hit == pytest.approx([10.0, 0.0, 0.0])


def test_ray_parallel_or_away_from_plane_returns_none():
    origin, normal = np.array([0, 0, 10.0]), np.array([0, 0, 1.0])
    assert intersect_ray_plane(origin, np.array([1.0, 0, 0]), np.zeros(3), normal) is None
    assert intersect_ray_plane(origin, np.array([0, 0, 1.0]), np.zeros(3), normal) is None


def test_plane_point_matches_pinhole_for_downward_camera():
    # Camera 300 mm above the table, looking straight down: z_C = -z_ref (rotation about x by pi).
    height = 300.0
    t_ref_cam = pose(rot_x(np.pi), [50.0, -20.0, height])
    u, v = 900.0, 200.0
    p = pixel_to_plane_point(u, v, K, t_ref_cam, 0.0)
    xc, yc, _ = pixel_to_camera_point(u, v, K, height)
    assert p == pytest.approx([50.0 + xc, -20.0 - yc, 0.0])


def test_plane_point_with_tilted_camera_reprojects_to_pixel():
    t_ref_cam = pose(rot_x(np.pi + np.deg2rad(5.0)), [0.0, 0.0, 300.0])
    u, v = 400.0, 500.0
    p = pixel_to_plane_point(u, v, K, t_ref_cam, 5.0)
    assert p[2] == pytest.approx(5.0)
    # Express the point in the camera frame and project it again.
    p_cam = np.linalg.inv(t_ref_cam) @ np.append(p, 1.0)
    uvw = K @ p_cam[:3]
    assert uvw[:2] / uvw[2] == pytest.approx([u, v])


def test_plane_above_camera_returns_none():
    t_ref_cam = pose(rot_x(np.pi), [0.0, 0.0, 300.0])
    assert pixel_to_plane_point(CX, CY, K, t_ref_cam, 400.0) is None
