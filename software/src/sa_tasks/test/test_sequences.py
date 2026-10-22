import numpy as np
import pytest

from sa_tasks import config, frames, sequences
from sa_tasks.interfaces import Candidate, MockGripper, MockRobot, Observation, View

CFG = config.load_tasks()
CFG["frames"]["target_correction_mm"] = [0.0, 0.0]  # the tests check the geometry
R_DOWN = config.rotation_down(CFG)
T_FT = CFG["frames"]["flange_to_tcp_mm"]
K = np.array([[2077.16, 0.0, 695.44], [0.0, 2083.73, 405.57], [0.0, 0.0, 1.0]])
TABLE_Z = 12.0
OBSERVE_NUT = frames.pose(R_DOWN, [480.0, 10.0, 480.0])
OBSERVE_STUD = frames.pose(R_DOWN, [420.0, 250.0, 480.0])
NUT = np.array([525.0, -15.0])
STUD = np.array([462.0, 248.0])


class FakePoses(config.Poses):
    def __init__(self):
        self.table_z_mm = TABLE_Z
        self.poses = {}
        self.set("observe_nut", OBSERVE_NUT)
        self.set("observe_stud", OBSERVE_STUD)


def pixel_of(point_xy, height, t_base_flange):
    """Where the camera sees a point on the plane table + height."""
    camera = t_base_flange @ config.camera_in_flange(CFG)
    uvw = K @ (frames.invert(camera) @ np.array([*point_xy, TABLE_Z + height, 1.0]))[:3]
    return uvw[0] / uvw[2], uvw[1] / uvw[2]


class SceneVision:
    """Objects (xy, shape_ok) on the stud centre plane, seen from wherever the robot is; the
    nut is seen from the nut observation pose."""

    def __init__(self, robot, objects=()):
        self.robot, self.objects, self.requests = robot, objects, []

    def prepare(self, target):
        pass

    def observe(self, target, plane_distance_mm):
        self.requests.append((target, plane_distance_mm))
        return Observation(*pixel_of(NUT, CFG["objects"]["nut"]["centre_height_mm"],
                                     self.robot.flange_pose()), K)

    def view(self, target, plane_distance_mm, lock):
        self.requests.append((target, plane_distance_mm))
        height = CFG["objects"]["stud"]["centre_height_mm"]
        candidates = []
        for xy, shape_ok in self.objects:
            u, v = pixel_of(xy, height, self.robot.flange_pose())
            if 0 <= u < 1280 and 0 <= v < 720:
                candidates.append(Candidate(u, v, 10.0, 0.71, shape_ok))
        return View(candidates, K)


def test_pick_spirals_onto_the_table_over_the_nut_and_closes():
    robot = MockRobot(frames.pose(R_DOWN, [300.0, 0.0, 500.0]), T_FT[2])
    gripper = MockGripper()
    record = {}
    sequences.pick(robot, gripper, SceneVision(robot), CFG, FakePoses(), record)
    assert record["nut_mm"][:2] == pytest.approx(NUT, abs=0.01)
    lowest = frames.tcp_position(min(robot.history, key=lambda t: t[2, 3]), T_FT)
    assert lowest[2] == pytest.approx(TABLE_Z - CFG["pick"]["press_depth_mm"])
    assert np.linalg.norm(lowest[:2] - NUT) == pytest.approx(CFG["pick"]["spiral"]["radius_mm"],
                                                             abs=0.01)
    assert gripper.commands == ["OPEN", "CLOSE"]
    assert robot.pose == pytest.approx(OBSERVE_NUT)


def test_stud_search_skips_an_object_that_is_no_stud_from_above():
    robot = MockRobot(OBSERVE_STUD.copy(), T_FT[2])
    vision = SceneVision(robot, [(STUD + [-30.0, 25.0], False), (STUD, True)])
    record = {}
    found = sequences.find_stud(robot, vision, CFG, TABLE_Z, OBSERVE_STUD, record)
    assert found["candidate"] == 2 and record["attempts"][0]["candidate"] == 1
    assert found["position_mm"][:2] == pytest.approx(STUD, abs=0.01)


def test_stud_search_without_a_stud_aborts():
    robot = MockRobot(OBSERVE_STUD.copy(), T_FT[2])
    with pytest.raises(sequences.SequenceError, match="none of the 1"):
        sequences.find_stud(robot, SceneVision(robot, [(STUD, False)]), CFG, TABLE_Z,
                            OBSERVE_STUD, {})


def run_place(contact_tcp_z=None, approach_only=False):
    robot = MockRobot(OBSERVE_NUT.copy(), T_FT[2], contact_tcp_z_mm=contact_tcp_z)
    gripper = MockGripper()
    record = {}
    sequences.place_and_screw(robot, gripper, SceneVision(robot, [(STUD, True)]), CFG,
                              FakePoses(), record, approach_only)
    return robot, gripper, record


def test_place_finds_contact_on_the_thread_and_screws():
    tip = TABLE_Z + CFG["objects"]["stud"]["top_height_mm"]
    robot, gripper, record = run_place(contact_tcp_z=tip - 1.0)
    assert record["contact_tcp_mm"][:2] == pytest.approx(STUD, abs=0.01)
    assert tip - 5.0 <= record["contact_tcp_mm"][2] <= tip
    assert gripper.commands == ["ROT", "ROT", "OPEN"]  # last ROT pressing down
    assert robot.pose == pytest.approx(OBSERVE_STUD)


@pytest.mark.parametrize("screw_anyway", [True, False])
def test_place_without_contact_screws_or_aborts(monkeypatch, screw_anyway):
    monkeypatch.setitem(CFG["place"], "screw_without_contact", screw_anyway)
    if screw_anyway:
        _, gripper, record = run_place()
        assert record["contact"] is False and gripper.commands == ["ROT", "ROT", "OPEN"]  # last ROT pressing down
    else:
        with pytest.raises(sequences.SequenceError, match="no contact"):
            run_place()


def test_approach_only_stops_above_the_thread_tip():
    _, gripper, record = run_place(approach_only=True)
    tip = TABLE_Z + CFG["objects"]["stud"]["top_height_mm"]
    assert record["approach_reached_tcp_mm"] == pytest.approx(
        [*STUD, tip + CFG["place"]["approach_clearance_mm"]], abs=0.01)
    assert gripper.commands == []


class LaggingRobot(MockRobot):
    """Stops 4 mm short in y towards y = 0, like the impedance controller (22.10.)."""

    def follow(self, path, stop_on_contact_n=None):
        contact = super().follow(path, stop_on_contact_n)
        self.pose[1, 3] -= 4.0 * np.sign(self.pose[1, 3])
        return contact


@pytest.mark.parametrize("y", [-200.0, 200.0])
def test_precise_move_removes_the_steady_state_error(y):
    robot = LaggingRobot(OBSERVE_NUT, T_FT[2])
    goal = frames.pose(R_DOWN, [450.0, y, 100.0])
    sequences.move_to(robot, goal, CFG)
    assert abs(robot.pose[1, 3] - y) == pytest.approx(4.0)
    reference = sequences.move_to(robot, goal, CFG, precise=True)
    assert robot.pose[:3, 3] == pytest.approx(goal[:3, 3], abs=CFG["robot"]["fine_positioning"]
                                              ["tolerance_mm"])
    assert reference[1, 3] - y == pytest.approx(4.0 * np.sign(y), abs=0.5)
