import copy

import numpy as np
import pytest
import yaml

from sa_tasks import config, frames

CFG = config.load_tasks()
R_DOWN = config.rotation_down(CFG)
K = np.array([[2077.16, 0.0, 695.44], [0.0, 2083.73, 405.57], [0.0, 0.0, 1.0]])


def test_quaternion_round_trip():
    rng = np.random.default_rng(1)
    for q in rng.normal(size=(20, 4)):
        q = q / np.linalg.norm(q) * np.sign(q[3])
        assert frames.matrix_to_quaternion(frames.quaternion_to_matrix(q)) == pytest.approx(q)


def test_flange_from_the_robot_state_puts_the_camera_in_front():
    # Franka Hand in Desk: L_T_EE = Rz(-45 deg), 103.4 mm along z; O_T_EE = (1, 0, 0, 0)
    t_link8_ee = frames.pose(frames.rotation_z(-45.0), [0, 0, 103.4])
    t_base_ee = frames.pose(R_DOWN, [500.0, 0.0, 400.0])
    flange, t_flange_ee = frames.flange_from_state(t_base_ee, t_link8_ee,
                                                   config.link8_to_flange(CFG))
    assert flange[:3, :3] == pytest.approx(R_DOWN)
    assert flange[:3, 3] == pytest.approx([500.0, 0.0, 503.4])
    assert flange @ t_flange_ee == pytest.approx(t_base_ee)
    camera = flange @ config.camera_in_flange(CFG)
    assert camera[:3, 3] == pytest.approx([544.4, 0.0, 503.4 - 57.223 + 9.0])
    # looks down; the tilt about x_F (= x_B here) moves the axis towards +y_B, the one about
    # y_F towards +x_B; the rotation about the optical axis leaves the axis alone
    tilt_x, tilt_y = np.radians(CFG["frames"]["camera_tilt_in_flange_deg"])
    assert camera[:3, 2] == pytest.approx([np.sin(tilt_y), np.sin(tilt_x) * np.cos(tilt_y),
                                           -np.cos(tilt_x) * np.cos(tilt_y)])


def test_object_in_base_inverts_the_projection_also_for_a_tilted_camera():
    tilt = frames.pose(frames.quaternion_to_matrix([np.sin(0.02), 0, 0, np.cos(0.02)]), [0, 0, 0])
    camera = frames.pose(R_DOWN, [450.0, 30.0, 420.0]) @ tilt @ config.camera_in_flange(CFG)
    nut = np.array([520.0, -10.0, 4.0])
    uvw = K @ (frames.invert(camera) @ np.append(nut, 1.0))[:3]
    assert frames.object_in_base(uvw[0] / uvw[2], uvw[1] / uvw[2], K, camera, 4.0) == \
        pytest.approx(nut)


def test_contradicting_cad_offsets_are_refused(tmp_path):
    cfg = copy.deepcopy(CFG)
    cfg["frames"]["flange_to_tcp_mm"] = [0.0, 0.0, 120.0]
    path = tmp_path / "tasks.yaml"
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="do not close"):
        config.load_tasks(path)


def test_tilt_shift_of_a_tilted_flange():
    down = frames.quaternion_to_matrix([1.0, 0.0, 0.0, 0.0])
    a = np.radians(1.0)
    tilt_about_x = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(a), -np.sin(a)], [0.0, np.sin(a), np.cos(a)]])
    shift = frames.tilt_shift_mm(frames.pose(tilt_about_x @ down, [0.0, 0.0, 0.0]), down, [0, 0, 100.0])
    assert abs(np.linalg.norm(shift[:2]) - 100.0 * np.sin(a)) < 1e-6
    assert np.allclose(frames.tilt_shift_mm(frames.pose(down, [1.0, 2.0, 3.0]), down, [0, 0, 100.0]), 0.0)
