"""Reference paths for the impedance controller. Pure NumPy, no ROS.

docs/THEORY.md, Section 5. A move is one continuous minimum-jerk motion. To come
down onto the table without sweeping over the nut, the horizontal part finishes
early ("xy leads"): x and y follow sigma(t / (lambda T)), z follows sigma(t / T),
so the last part of the move is (nearly) vertical. Moving up, z leads instead.
"""
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from sa_tasks import frames

# Peak speed of a minimum-jerk profile divided by its mean speed
MIN_JERK_PEAK_FACTOR = 1.875
LEAD_XY = "xy"
LEAD_Z = "z"


def min_jerk(s: float) -> float:
    """Minimum-jerk profile sigma(s) = 10 s^3 - 15 s^4 + 6 s^5, clamped to [0, 1]."""
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10.0 - 15.0 * s + 6.0 * s * s)


@dataclass(frozen=True)
class MotionLimits:
    max_speed_mm_s: float        # peak speed of every axis group
    max_angular_speed_deg_s: float
    lead_fraction: float         # lambda: the leading axes finish at lambda * T
    min_duration_s: float


@dataclass
class Waypoint:
    time_s: float
    t_base_flange: np.ndarray


def choose_lead(start: np.ndarray, goal: np.ndarray) -> str:
    """Down: xy first (arrive vertically). Up: z first (leave vertically)."""
    return LEAD_XY if goal[2, 3] < start[2, 3] else LEAD_Z


def duration(start: np.ndarray, goal: np.ndarray, limits: MotionLimits, lead: str) -> float:
    """Shortest T for which no axis group exceeds its peak speed."""
    delta = goal[:3, 3] - start[:3, 3]
    xy, z = float(np.linalg.norm(delta[:2])), abs(float(delta[2]))
    angle = frames.rotation_angle_deg(start[:3, :3], goal[:3, :3])
    lead_xy = limits.lead_fraction if lead == LEAD_XY else 1.0
    lead_z = limits.lead_fraction if lead == LEAD_Z else 1.0
    times = [
        MIN_JERK_PEAK_FACTOR * xy / (lead_xy * limits.max_speed_mm_s),
        MIN_JERK_PEAK_FACTOR * z / (lead_z * limits.max_speed_mm_s),
        MIN_JERK_PEAK_FACTOR * angle / limits.max_angular_speed_deg_s,
        limits.min_duration_s,
    ]
    return max(times)


def plan_move(start: np.ndarray, goal: np.ndarray, limits: MotionLimits, rate_hz: float,
              lead: Optional[str] = None) -> List[Waypoint]:
    """Flange poses from start to goal at rate_hz, ending exactly at goal."""
    lead = lead or choose_lead(start, goal)
    total = duration(start, goal, limits, lead)
    p0, p1 = start[:3, 3], goal[:3, 3]
    q0 = frames.matrix_to_quaternion(start[:3, :3])
    q1 = frames.matrix_to_quaternion(goal[:3, :3])
    steps = max(int(np.ceil(total * rate_hz)), 1)
    path = []
    for i in range(1, steps + 1):
        t = total * i / steps
        slow = min_jerk(t / total)
        fast = min_jerk(t / (limits.lead_fraction * total))
        s_xy, s_z = (fast, slow) if lead == LEAD_XY else (slow, fast)
        position = np.array([p0[0] + s_xy * (p1[0] - p0[0]), p0[1] + s_xy * (p1[1] - p0[1]),
                             p0[2] + s_z * (p1[2] - p0[2])])
        rotation = frames.quaternion_to_matrix(frames.slerp(q0, q1, slow))
        path.append(Waypoint(t, frames.pose(rotation, position)))
    path[-1].t_base_flange = goal.copy()
    return path


def straight_down(start: np.ndarray, depth_mm: float, speed_mm_s: float,
                  rate_hz: float) -> List[Waypoint]:
    """Constant-speed vertical path (contact search), same orientation."""
    steps = max(int(np.ceil(depth_mm / speed_mm_s * rate_hz)), 1)
    path = []
    for i in range(1, steps + 1):
        t = start.copy()
        t[2, 3] -= depth_mm * i / steps
        path.append(Waypoint(i / rate_hz, t))
    return path
