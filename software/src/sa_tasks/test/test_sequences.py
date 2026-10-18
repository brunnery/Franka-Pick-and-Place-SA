import copy
import math

import numpy as np
import pytest

from sa_tasks import config, frames, sequences
from sa_tasks.interfaces import MockGripper, MockRobot, MockVision
from sa_vision import geometry

CFG = config.load_tasks()
# The tests check the geometry; the measured correction of tasks.yaml would shift every
# expected position (test_target_correction_is_added covers it separately)
CFG["frames"]["target_correction_mm"] = [0.0, 0.0]
R_DOWN = config.rotation_down(CFG)
T_FT = CFG["frames"]["flange_to_tcp_mm"]
K = geometry.camera_matrix([2077.16, 0.0, 695.44, 0.0, 2083.73, 405.57, 0.0, 0.0, 1.0])
TABLE_Z = 12.0
OBSERVE_NUT = frames.pose(R_DOWN, [480.0, 10.0, 480.0])
OBSERVE_STUD = frames.pose(R_DOWN, [420.0, 250.0, 480.0])
NUT = np.array([525.0, -15.0])
STUD = np.array([462.0, 248.0])


class FakePoses(config.Poses):
    def __init__(self, observe_nut=OBSERVE_NUT, observe_stud=OBSERVE_STUD, table_z=TABLE_Z):
        self.path = None
        self.table_z_mm = table_z
        self.gripper_tilt_deg = None
        self.gripper_tilt_samples = []
        self.poses = {}
        self.set("observe_nut", observe_nut)
        self.set("observe_stud", observe_stud)


def pixel_of(point_xy, height, observe):
    """Where the camera at the observation pose sees a point on the plane table + height."""
    camera = observe @ config.camera_in_flange(CFG)
    p = (frames.invert(camera) @ np.array([*point_xy, TABLE_Z + height, 1.0]))[:3]
    uvw = K @ p
    return uvw[0] / uvw[2], uvw[1] / uvw[2]


def run_pick(vision=None, poses=None, cfg=CFG):
    robot = MockRobot(frames.pose(R_DOWN, [300.0, 0.0, 500.0]), T_FT[2])
    gripper = MockGripper()
    if vision is None:
        u, v = pixel_of(NUT, CFG["objects"]["nut"]["centre_height_mm"], OBSERVE_NUT)
        vision = MockVision(u, v, K)
    result = sequences.pick(robot, gripper, vision, cfg, poses or FakePoses())
    return robot, gripper, vision, result


def test_pick_puts_the_tcp_over_the_nut_flat_on_the_table():
    robot, gripper, vision, result = run_pick()
    assert result["nut"]["position_mm"][:2] == pytest.approx(NUT, abs=1e-6)
    lowest = min(robot.history, key=lambda t: t[2, 3])
    tcp = frames.tcp_position(lowest, T_FT)
    assert tcp == pytest.approx([*NUT, TABLE_Z - CFG["pick"]["press_depth_mm"]], abs=1e-6)
    assert frames.tilt_deg(lowest[:3, :3]) == pytest.approx(0.0, abs=1e-6)
    assert gripper.commands == ["OPEN", "CLOSE"]
    assert robot.pose == pytest.approx(OBSERVE_NUT)  # back at the start pose


def test_locate_only_does_not_descend():
    robot = MockRobot(frames.pose(R_DOWN, [300.0, 0.0, 500.0]), T_FT[2])
    u, v = pixel_of(NUT, CFG["objects"]["nut"]["centre_height_mm"], OBSERVE_NUT)
    result = sequences.pick(robot, MockGripper(), MockVision(u, v, K), CFG, FakePoses(),
                            locate_only=True)
    assert result["nut"]["position_mm"][:2] == pytest.approx(NUT, abs=1e-6)
    assert min(t[2, 3] for t in robot.history) == pytest.approx(OBSERVE_NUT[2, 3])


def test_pick_records_where_the_gripper_landed():
    _, _, _, result = run_pick()
    assert result["landed_tcp_mm"][:2] == pytest.approx(NUT, abs=1e-6)
    assert result["landed_above_table_mm"] == pytest.approx(-CFG["pick"]["press_depth_mm"])


def test_pick_tells_vision_the_distance_to_the_nut_top_face():
    _, _, vision, _ = run_pick()
    camera_z = OBSERVE_NUT[2, 3] - 57.223 + CFG["frames"]["projection_centre_above_housing_mm"]
    target, distance = vision.requests[0]
    assert target == "nut"
    assert distance == pytest.approx(camera_z - TABLE_Z - CFG["objects"]["nut"]["height_mm"])


def test_pick_descends_vertically_at_the_end():
    robot, _, _, _ = run_pick()
    lowest = int(np.argmin([t[2, 3] for t in robot.history]))
    descent = robot.history[:lowest + 1]
    low = [t for t in descent if t[2, 3] < TABLE_Z + T_FT[2] + 30.0]
    assert low and all(np.allclose(t[:2, 3], low[-1][:2, 3]) for t in low)


def test_target_correction_is_added():
    cfg = copy.deepcopy(CFG)
    cfg["frames"]["target_correction_mm"] = [1.5, -0.5]
    _, _, _, result = run_pick(cfg=cfg)
    assert result["nut"]["position_mm"][:2] == pytest.approx(NUT + [1.5, -0.5], abs=1e-6)


def test_detection_outside_the_area_is_refused():
    far = NUT + [0.0, 200.0]
    u, v = pixel_of(far, 4.0, OBSERVE_NUT)
    with pytest.raises(sequences.SequenceError, match="outside the work area"):
        run_pick(vision=MockVision(u, v, K))


def test_nothing_found_aborts_before_moving_down():
    with pytest.raises(RuntimeError, match="no nut found"):
        run_pick(vision=MockVision(math.nan, math.nan, K))


def test_both_observation_poses_use_the_height_of_the_nut_pose():
    poses = FakePoses(observe_stud=frames.pose(R_DOWN, [420.0, 250.0, 455.0]))
    stud = sequences.observation_pose(poses, "observe_stud", CFG)
    assert stud[:3, 3] == pytest.approx([420.0, 250.0, OBSERVE_NUT[2, 3]])
    cfg = copy.deepcopy(CFG)
    cfg["robot"]["observation_height_from"] = ""
    assert sequences.observation_pose(poses, "observe_stud", cfg)[2, 3] == pytest.approx(455.0)


def test_observation_pose_uses_the_fixed_orientation_whatever_was_taught():
    yawed = frames.pose(R_DOWN @ frames.rotation_z(-45.0)
                        @ frames.quaternion_to_matrix([np.sin(0.03), 0, 0, np.cos(0.03)]),
                        OBSERVE_NUT[:3, 3])
    observe = sequences.observation_pose(FakePoses(observe_nut=yawed), "observe_nut", CFG)
    assert frames.matrix_to_quaternion(observe[:3, :3]) == pytest.approx([1, 0, 0, 0])
    assert observe[:3, 3] == pytest.approx(OBSERVE_NUT[:3, 3])


def tilted_cfg_and_poses(tilt_deg):
    cfg = copy.deepcopy(CFG)
    cfg["frames"]["use_gripper_tilt"] = True
    poses = FakePoses()
    poses.gripper_tilt_deg = list(tilt_deg)
    return cfg, poses


def test_flat_sample_measures_the_mount_tilt():
    # Bottom plate flat on the table, gripper tilted 1.5 / -0.8 deg against the flange:
    # then the flange itself is tilted by the inverse
    t_flange_tool = frames.matrix_from_rotation_vector_deg([1.5, -0.8, 0.0])
    flange = frames.pose(R_DOWN @ t_flange_tool.T, [480.0, 0.0, 130.0])
    poses = FakePoses()
    sample = poses.add_tilt_sample(flange)
    # The levelled reference keeps the yaw of x_F, which leaves a second-order z rotation
    assert sample == pytest.approx([1.5, -0.8, 0.0], abs=0.02)
    assert poses.gripper_tilt_deg == pytest.approx([1.5, -0.8, 0.0], abs=0.02)


def test_with_gripper_tilt_the_gripper_lands_level_and_on_target():
    cfg, poses = tilted_cfg_and_poses([1.5, -0.8, 0.0])
    t_flange_tool = config.tool_in_flange(cfg, poses)
    robot = MockRobot(frames.pose(R_DOWN, [300.0, 0.0, 500.0]), T_FT[2])
    observe = OBSERVE_NUT @ t_flange_tool  # where the camera really is when G is level
    u, v = pixel_of(NUT, cfg["objects"]["nut"]["centre_height_mm"], OBSERVE_NUT)
    result = sequences.pick(robot, MockGripper(), MockVision(u, v, K), cfg, poses)
    assert result["nut"]["position_mm"][:2] == pytest.approx(NUT, abs=1e-6)
    lowest = min(robot.history, key=lambda t: t[2, 3])
    tool = lowest @ t_flange_tool
    assert frames.tilt_deg(tool[:3, :3]) == pytest.approx(0.0, abs=1e-6)   # plate level
    assert frames.tilt_deg(lowest[:3, :3]) > 1.0                           # flange tilted
    assert frames.tcp_position(tool, T_FT)[:2] == pytest.approx(NUT, abs=1e-6)
    del observe


def test_gripper_tilt_switched_on_without_measurement_is_refused():
    cfg, poses = tilted_cfg_and_poses([0.0, 0.0, 0.0])
    poses.gripper_tilt_deg = None
    with pytest.raises(RuntimeError, match="no tilt measured"):
        sequences.pick(MockRobot(OBSERVE_NUT.copy(), T_FT[2]), MockGripper(),
                       MockVision(640.0, 360.0, K), cfg, poses)


def test_untaught_pose_and_tilted_teach_are_refused():
    poses = FakePoses()
    poses.poses["observe_nut"] = None
    with pytest.raises(RuntimeError, match="not taught"):
        run_pick(poses=poses)
    tilted = OBSERVE_NUT.copy()
    tilted[:3, :3] = R_DOWN @ frames.quaternion_to_matrix([np.sin(0.1), 0, 0, np.cos(0.1)])  # 11.5 deg
    with pytest.raises(sequences.SequenceError, match="tilted"):
        run_pick(poses=FakePoses(observe_nut=tilted))


def run_place(contact_tcp_z):
    robot = MockRobot(OBSERVE_NUT.copy(), T_FT[2], contact_tcp_z_mm=contact_tcp_z)
    gripper = MockGripper()
    u, v = pixel_of(STUD, CFG["objects"]["stud"]["base_height_mm"], OBSERVE_STUD)
    vision = MockVision(u, v, K)
    return robot, gripper, vision, sequences.place_and_screw(robot, gripper, vision, CFG,
                                                             FakePoses())


def test_place_finds_contact_on_the_thread_and_screws():
    thread_tip = TABLE_Z + CFG["objects"]["stud"]["top_height_mm"]
    robot, gripper, vision, result = run_place(contact_tcp_z=thread_tip - 1.0)
    assert vision.requests[0][0] == "stud"
    assert result["stud"]["position_mm"][:2] == pytest.approx(STUD, abs=1e-6)
    assert result["contact_tcp_mm"][:2] == pytest.approx(STUD, abs=1e-6)
    assert thread_tip - 1.0 - 4.0 / 1.5 - 0.5 <= result["contact_tcp_mm"][2] <= thread_tip
    assert gripper.commands == ["ROT", "OPEN"]
    assert robot.mode == CFG["robot"]["modes"]["normal"]
    assert robot.pose == pytest.approx(OBSERVE_STUD)


def test_place_without_contact_aborts_and_backs_off():
    robot = None
    with pytest.raises(sequences.SequenceError, match="no contact"):
        robot, *_ = run_place(contact_tcp_z=None)
