import numpy as np
import pytest

from sa_tasks import config, frames
from sa_tasks.trajectory import MotionLimits, plan_move, spiral_down

R_DOWN = config.rotation_down(config.load_tasks())
LIMITS = MotionLimits(max_speed_mm_s=80.0, max_angular_speed_deg_s=30.0, lead_fraction=0.6,
                      min_duration_s=1.0)
HIGH = frames.pose(R_DOWN, [450.0, 0.0, 450.0])
LOW = frames.pose(R_DOWN, [520.0, -40.0, 120.0])


def test_move_ends_at_the_goal_within_the_speed_limit():
    path = plan_move(HIGH, LOW, LIMITS, 100.0)
    assert path[-1].t_base_flange == pytest.approx(LOW)
    p = np.array([w.t_base_flange[:3, 3] for w in path])
    v = np.diff(p, axis=0) / np.diff([w.time_s for w in path])[:, None]
    assert np.linalg.norm(v[:, :2], axis=1).max() <= LIMITS.max_speed_mm_s * 1.01
    assert np.abs(v[:, 2]).max() <= LIMITS.max_speed_mm_s * 1.01


def test_going_down_the_end_of_the_move_is_vertical():
    path = plan_move(HIGH, LOW, LIMITS, 100.0)
    late = [w for w in path if w.time_s >= LIMITS.lead_fraction * path[-1].time_s]
    assert all(np.allclose(w.t_base_flange[:2, 3], LOW[:2, 3]) for w in late)
    assert late[0].t_base_flange[2, 3] > LOW[2, 3] + 10.0


def test_spiral_grows_outward_while_descending():
    path = spiral_down(LOW, 12.0, 6.0, 2.0, 4.0, 100.0)
    radii = [np.linalg.norm(w.t_base_flange[:2, 3] - LOW[:2, 3]) for w in path]
    assert radii[-1] == pytest.approx(6.0) and path[-1].t_base_flange[2, 3] == pytest.approx(108.0)
    assert all(b.t_base_flange[2, 3] <= a.t_base_flange[2, 3] for a, b in zip(path, path[1:]))
