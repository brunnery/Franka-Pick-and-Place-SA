import numpy as np
import pytest

from sa_tasks import config, frames
from sa_tasks.trajectory import (LEAD_XY, LEAD_Z, MotionLimits, choose_lead, min_jerk,
                                 chain, plan_move, spiral_down, straight_down)

R_DOWN = config.rotation_down(config.load_tasks())
LIMITS = MotionLimits(max_speed_mm_s=80.0, max_angular_speed_deg_s=30.0, lead_fraction=0.6,
                      min_duration_s=1.0)
RATE = 100.0
HIGH = frames.pose(R_DOWN, [450.0, 0.0, 450.0])
LOW = frames.pose(R_DOWN, [520.0, -40.0, 120.0])


def speeds(path):
    p = np.array([w.t_base_flange[:3, 3] for w in path])
    t = np.array([w.time_s for w in path])
    v = np.diff(p, axis=0) / np.diff(t)[:, None]
    return np.linalg.norm(v[:, :2], axis=1), np.abs(v[:, 2])


def test_min_jerk_profile():
    assert min_jerk(0.0) == 0.0 and min_jerk(1.0) == 1.0 and min_jerk(0.5) == pytest.approx(0.5)
    assert min_jerk(-1.0) == 0.0 and min_jerk(2.0) == 1.0


def test_lead_is_xy_going_down_and_z_going_up():
    assert choose_lead(HIGH, LOW) == LEAD_XY
    assert choose_lead(LOW, HIGH) == LEAD_Z


def test_move_ends_exactly_at_the_goal_and_respects_the_speed_limit():
    path = plan_move(HIGH, LOW, LIMITS, RATE)
    assert path[-1].t_base_flange == pytest.approx(LOW)
    xy, z = speeds(path)
    assert xy.max() <= LIMITS.max_speed_mm_s * 1.01
    assert z.max() <= LIMITS.max_speed_mm_s * 1.01


def test_going_down_the_xy_motion_ends_before_the_descent():
    path = plan_move(HIGH, LOW, LIMITS, RATE)
    total = path[-1].time_s
    done = [w for w in path if w.time_s >= LIMITS.lead_fraction * total]
    # From lambda * T on, only z changes: the last part of the move is vertical
    assert all(np.allclose(w.t_base_flange[:2, 3], LOW[:2, 3]) for w in done)
    assert done[0].t_base_flange[2, 3] > LOW[2, 3] + 10.0


def test_going_up_the_lift_ends_before_the_xy_motion():
    path = plan_move(LOW, HIGH, LIMITS, RATE)
    total = path[-1].time_s
    lifted = [w for w in path if w.time_s >= LIMITS.lead_fraction * total]
    assert all(w.t_base_flange[2, 3] == pytest.approx(HIGH[2, 3]) for w in lifted)
    assert not np.allclose(lifted[0].t_base_flange[:2, 3], HIGH[:2, 3])


def test_orientation_is_interpolated_to_the_goal():
    tilted = HIGH.copy()
    tilted[:3, :3] = R_DOWN @ frames.quaternion_to_matrix([0, 0, np.sin(0.2), np.cos(0.2)])
    path = plan_move(tilted, LOW, LIMITS, RATE)
    assert frames.rotation_angle_deg(path[-1].t_base_flange[:3, :3], R_DOWN) == pytest.approx(0.0)


def test_straight_down():
    path = straight_down(LOW, 10.0, 5.0, RATE)
    assert path[-1].t_base_flange[2, 3] == pytest.approx(LOW[2, 3] - 10.0)
    assert path[-1].time_s == pytest.approx(2.0)
    assert all(np.allclose(w.t_base_flange[:2, 3], LOW[:2, 3]) for w in path)


def test_spiral_down_grows_outward_while_descending_and_ends_at_rest():
    path = spiral_down(LOW, depth_mm=12.0, radius_mm=6.0, turns=2.0, duration_s=4.0,
                       rate_hz=RATE)
    p = np.array([w.t_base_flange[:3, 3] for w in path])
    radius = np.linalg.norm(p[:, :2] - LOW[:2, 3], axis=1)
    assert radius[-1] == pytest.approx(6.0) and np.all(np.diff(radius) >= -1e-9)
    assert p[-1, 2] == pytest.approx(LOW[2, 3] - 12.0) and np.all(np.diff(p[:, 2]) <= 1e-9)
    assert path[-1].time_s == pytest.approx(4.0)
    assert all(np.allclose(w.t_base_flange[:3, :3], LOW[:3, :3]) for w in path)
    xy, z = speeds(path)
    assert xy[0] < 1.0 and xy[-1] < 1.0  # starts and ends at rest


def test_chain_shifts_the_second_path_in_time():
    first = plan_move(HIGH, LOW, LIMITS, RATE)
    second = spiral_down(LOW, 5.0, 3.0, 1.0, 2.0, RATE)
    path = chain(first, second)
    assert len(path) == len(first) + len(second)
    times = [w.time_s for w in path]
    assert np.all(np.diff(times) > 0)
    assert times[-1] == pytest.approx(first[-1].time_s + 2.0)
