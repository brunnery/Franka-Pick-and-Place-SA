import numpy as np
import pytest

from sa_tasks import config, frames
from sa_vision import geometry

CFG = config.load_tasks()
R_DOWN = config.rotation_down(CFG)
K = geometry.camera_matrix([2077.16, 0.0, 695.44, 0.0, 2083.73, 405.57, 0.0, 0.0, 1.0])


def project(point_base, t_base_camera):
    """Pixel of a base-frame point (ideal pinhole)."""
    p = (frames.invert(t_base_camera) @ np.append(point_base, 1.0))[:3]
    uvw = K @ p
    return uvw[0] / uvw[2], uvw[1] / uvw[2]


def test_quaternion_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        q = -q if q[3] < 0 else q
        assert frames.matrix_to_quaternion(frames.quaternion_to_matrix(q)) == pytest.approx(q)


def test_rotation_down_matches_the_documented_axes():
    # x_F = +y_B, y_F = +x_B, z_F = -z_B (CLAUDE.md)
    assert R_DOWN[:, 0] == pytest.approx([0, 1, 0])
    assert R_DOWN[:, 1] == pytest.approx([1, 0, 0])
    assert R_DOWN[:, 2] == pytest.approx([0, 0, -1])
    assert frames.tilt_deg(R_DOWN) == pytest.approx(0.0)


def test_camera_sits_in_front_of_the_flange_and_looks_down():
    flange = frames.pose(R_DOWN, [500.0, 0.0, 400.0])
    camera = flange @ config.camera_in_flange(CFG)
    assert camera[0, 3] == pytest.approx(544.4)            # +x_base, 44.4 mm
    assert camera[2, 3] == pytest.approx(400.0 - 57.223 + 9.0)
    assert camera[:3, 2] == pytest.approx([0, 0, -1])      # z_C down


def test_camera_axes_agree_with_the_vision_axis_mapping():
    # A point to the right in the image (+x_C) must lie at -y_base, and further down in
    # the image (+y_C) at -x_base: sa_vision.geometry.camera_to_robot_xy
    camera = frames.pose(R_DOWN, [0, 0, 300.0]) @ config.camera_in_flange(CFG)
    for x_c, y_c in [(10.0, 0.0), (0.0, 10.0), (7.0, -3.0)]:
        p_base = camera @ np.array([x_c, y_c, 100.0, 1.0])
        expected = geometry.camera_to_robot_xy(x_c, y_c)
        assert (p_base[0] - camera[0, 3], p_base[1] - camera[1, 3]) == pytest.approx(expected)


def test_object_in_base_inverts_the_projection_also_for_a_tilted_camera():
    tilt = frames.pose(frames.quaternion_to_matrix([np.sin(0.02), 0, 0, np.cos(0.02)]), [0, 0, 0])
    flange = frames.pose(R_DOWN, [450.0, 30.0, 420.0]) @ tilt
    camera = flange @ config.camera_in_flange(CFG)
    nut = np.array([520.0, -10.0, 4.0])
    u, v = project(nut, camera)
    assert frames.object_in_base(u, v, K, camera, 4.0) == pytest.approx(nut, abs=1e-6)


def test_flange_for_tcp_puts_the_tcp_on_the_target():
    tcp = [500.0, 20.0, -2.0]
    flange = frames.flange_for_tcp(tcp, R_DOWN, CFG["frames"]["flange_to_tcp_mm"])
    assert flange[2, 3] == pytest.approx(-2.0 + 118.823)
    assert frames.tcp_position(flange, CFG["frames"]["flange_to_tcp_mm"]) == pytest.approx(tcp)


def test_end_effector_round_trip():
    f_t_ee = frames.pose(frames.quaternion_to_matrix([0, 0, np.sin(-np.pi / 8),
                                                      np.cos(-np.pi / 8)]), [0, 0, 103.4])
    flange = frames.pose(R_DOWN, [400.0, 0.0, 300.0])
    ee = frames.end_effector_from_flange(flange, f_t_ee)
    assert frames.flange_from_end_effector(ee, f_t_ee) == pytest.approx(flange)


def test_distance_along_axis_and_footprint():
    camera = frames.pose(R_DOWN, [0, 0, 400.0]) @ config.camera_in_flange(CFG)
    d = frames.distance_along_axis(camera, 8.0)
    assert d == pytest.approx(400.0 - 57.223 + 9.0 - 8.0)
    width, height = frames.footprint_mm(d, K, (1280, 720))
    assert width == pytest.approx(1280 * d / K[0, 0])
