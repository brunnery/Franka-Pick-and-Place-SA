"""Step 1 (pick the nut) and step 2 (place it on the stud and screw it on). No ROS.

docs/PLAN.md Section 4, docs/THEORY.md Sections 2, 3 and 5. Only the Robot, Gripper and
Vision interfaces are used, so the sequences run against mocks.
"""
import logging
import threading
from typing import Callable, Optional

import numpy as np

from sa_tasks import config, frames
from sa_tasks.interfaces import Gripper, Robot, Vision
from sa_tasks.trajectory import (LEAD_XY, LEAD_Z, Waypoint, chain, plan_move, spiral_down,
                                 straight_down)

log = logging.getLogger("sa_tasks")

TARGET_NUT = "nut"
TARGET_STUD = "stud"


class SequenceError(RuntimeError):
    """A step failed; the robot stays where it is (no automatic recovery)."""


def rounded(values) -> list:
    return [round(float(v), 2) for v in values]


def move_to(robot: Robot, goal: np.ndarray, cfg: dict, lead: Optional[str] = None) -> None:
    """One continuous move of the flange from where it is to goal, then wait for arrival."""
    robot.follow(plan_move(robot.flange_pose(), goal, config.motion_limits(cfg),
                           cfg["robot"]["stream_rate_hz"], lead))
    robot.wait_until_arrived(goal, cfg["robot"]["arrival_tolerance_mm"])


def move_while(robot: Robot, goal: np.ndarray, cfg: dict, action: Callable[[], None]) -> None:
    """move_to while action() runs alongside (e.g. the iris opening); errors of both raise."""
    errors = []

    def run():
        try:
            action()
        except Exception as error:  # noqa: BLE001 - re-raised below
            errors.append(error)
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    move_to(robot, goal, cfg)
    worker.join()
    if errors:
        raise errors[0]


def observation_pose(poses: config.Poses, name: str, cfg: dict) -> np.ndarray:
    """Taught position with the fixed orientation. With robot.observation_height_from set,
    z comes from that pose (both observation poses share one height, same image scale)."""
    position = poses.get(name)[:3, 3].copy()
    if cfg["robot"]["observation_height_from"]:
        position[2] = poses.get(cfg["robot"]["observation_height_from"])[2, 3]
    return frames.pose(config.rotation_down(cfg), position)


def camera_and_distance(robot: Robot, cfg: dict, plane_z_mm: float) -> tuple:
    t_base_camera = robot.flange_pose() @ config.camera_in_flange(cfg)
    return t_base_camera, frames.distance_along_axis(t_base_camera, plane_z_mm)


def locate_nut(robot: Robot, vision: Vision, cfg: dict, table_z: float) -> list:
    """Nut position in the base frame (incl. target_correction_mm) from the current pose."""
    nut = cfg["objects"]["nut"]
    camera, distance = camera_and_distance(robot, cfg, table_z + nut["height_mm"])
    seen = vision.observe(TARGET_NUT, distance)
    plane_z = table_z + nut["centre_height_mm"]
    point = frames.object_in_base(seen.u_undistorted, seen.v_undistorted, seen.k, camera, plane_z)
    axis = frames.object_in_base(seen.k[0, 2], seen.k[1, 2], seen.k, camera, plane_z)
    offset = float(np.linalg.norm(point[:2] - axis[:2]))
    if offset > cfg["vision"]["max_offset_from_axis_mm"]:
        raise SequenceError(f"nut found {offset:.0f} mm from the camera axis, outside the "
                            "work area")
    point[:2] += cfg["frames"]["target_correction_mm"]
    log.info("nut at x %.1f y %.1f mm, %.0f mm from the camera axis", point[0], point[1], offset)
    return rounded(point)


def stud_view(robot: Robot, vision: Vision, cfg: dict, table_z: float, lock: bool) -> tuple:
    """All objects of one view (rank order of vision_node) on the stud's centre plane.
    Returns (camera axis point on that plane, [dict per object])."""
    stud = cfg["objects"]["stud"]
    camera, distance = camera_and_distance(robot, cfg, table_z + stud["base_height_mm"])
    view = vision.view(TARGET_STUD, distance, lock)
    plane_z = table_z + stud["centre_height_mm"]
    axis = frames.object_in_base(view.k[0, 2], view.k[1, 2], view.k, camera, plane_z)
    objects = []
    for rank, c in enumerate(view.candidates, 1):
        point = frames.object_in_base(c.u_undistorted, c.v_undistorted, view.k, camera, plane_z)
        offset = float(np.linalg.norm(point[:2] - axis[:2]))
        objects.append({"rank": rank, "position_mm": rounded(point),
                        "offset_from_axis_mm": round(offset, 2), "size_mm": round(c.size_mm, 2),
                        "width_ratio": round(c.width_ratio, 3), "shape_ok": bool(c.shape_ok)})
    return axis, objects


def find_stud(robot: Robot, vision: Vision, cfg: dict, table_z: float, observe: np.ndarray,
              record: dict) -> dict:
    """Stud search (THEORY 2.0): the tall stud is a clean square only from directly above.
    The objects locked at the observation pose are tried in rank order: the camera axis is
    moved over the object (corrected from the measured pose) until it is within
    centred_tolerance_mm, then the largest object within lost_radius_mm of the axis is judged
    (dark spots on the stud can be separate small blobs). The first square is the stud; its
    position comes from the centred image, where the parallax of the tall stud is negligible."""
    search = cfg["place"]["stud_search"]
    axis, seen = stud_view(robot, vision, cfg, table_z, lock=True)
    record["candidates"] = seen
    candidates = [c for c in seen if c["offset_from_axis_mm"]
                  <= cfg["vision"]["max_offset_from_axis_mm"]][:search["max_candidates"]]
    if not candidates:
        raise SequenceError(f"no object in the work area at the stud observation pose "
                            f"({len(seen)} in view)")
    attempts = record["attempts"] = []
    goal = observe.copy()  # last commanded pose = where `axis` was measured
    for candidate in candidates:
        position = np.asarray(candidate["position_mm"][:2])
        for move in range(1, search["max_centring_moves"] + 1):
            goal[:2, 3] += position - axis[:2]
            move_to(robot, goal, cfg)
            axis, now = stud_view(robot, vision, cfg, table_z, lock=False)
            near = [c for c in now if c["offset_from_axis_mm"] <= search["lost_radius_mm"]]
            nearest = max(near, key=lambda c: c["size_mm"]) if near else None
            attempts.append({"candidate": candidate["rank"], "move": move, "nearest": nearest,
                             "seen": now})
            if nearest is None:
                log.info("candidate #%d lost after centring move %d: %d object(s) in view, "
                         "nearest %s mm from the axis", candidate["rank"], move, len(now),
                         min((c["offset_from_axis_mm"] for c in now), default="-"))
                break
            position = np.asarray(nearest["position_mm"][:2])
            if nearest["offset_from_axis_mm"] > search["centred_tolerance_mm"]:
                continue
            if not nearest["shape_ok"]:
                log.info("candidate #%d from above: w %.2f, %.1f mm: not a stud",
                         candidate["rank"], nearest["width_ratio"], nearest["size_mm"])
                break
            target = np.asarray(nearest["position_mm"], float)
            target[:2] += cfg["frames"]["target_correction_mm"]
            log.info("stud = candidate #%d (w %.2f, %.1f mm) at x %.1f y %.1f mm after %d "
                     "move(s)", candidate["rank"], nearest["width_ratio"], nearest["size_mm"],
                     target[0], target[1], move)
            return {"position_mm": rounded(target), "centred_mm": nearest["position_mm"],
                    "observed_mm": candidate["position_mm"], "candidate": candidate["rank"]}
    raise SequenceError(f"none of the {len(candidates)} object(s) tried is a stud")


def pick(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict, poses: config.Poses,
         record: dict, back_up: bool = True) -> None:
    """Step 1: observation pose (the iris opens on the way) -> detect the nut -> one move down
    with a spiral search until the bottom plate rests on the table, the open iris over the
    nut -> close -> back up. back_up False (full run): stay on the table, the next move
    (to observe_stud, z leading) lifts the nut in one motion."""
    table_z = poses.table_z()
    observe = observation_pose(poses, "observe_nut", cfg)
    robot.set_mode(cfg["robot"]["modes"]["normal"])
    vision.prepare(TARGET_NUT)
    move_while(robot, observe, cfg, gripper.open)
    nut = record["nut_mm"] = locate_nut(robot, vision, cfg, table_z)

    # Down to just above the nut, then without a stop a spiral down onto the table: if the
    # target is off by up to the spiral radius, the plate slides over the nut until the open
    # iris passes over it and the nut drops in (THEORY 5)
    spiral = cfg["pick"]["spiral"]
    press = cfg["pick"]["press_depth_mm"]
    rate = cfg["robot"]["stream_rate_hz"]
    start = frames.flange_for_tcp([nut[0], nut[1], table_z + spiral["start_height_mm"]],
                                  config.rotation_down(cfg), cfg["frames"]["flange_to_tcp_mm"])
    search = spiral_down(start, spiral["start_height_mm"] + press, spiral["radius_mm"],
                         spiral["turns"], spiral["duration_s"], rate)
    robot.follow(chain(plan_move(robot.flange_pose(), start, config.motion_limits(cfg), rate,
                                 LEAD_XY), search))
    # If the iris misses, the plate stands on the nut instead of the table: accept that
    robot.wait_until_arrived(search[-1].t_base_flange, cfg["robot"]["arrival_tolerance_mm"]
                             + press + cfg["objects"]["nut"]["height_mm"])
    landed = frames.tcp_position(robot.flange_pose(), cfg["frames"]["flange_to_tcp_mm"])
    record["landed_tcp_mm"] = rounded(landed)
    log.info("landed: TCP %.1f mm above the table", landed[2] - table_z)
    gripper.close()
    if back_up:
        move_to(robot, observe, cfg, lead=LEAD_Z)


def position_check(found: dict, reached_tcp: np.ndarray) -> dict:
    """Stud xy at three stages (approach_only): 1 from the observation pose (off-axis),
    2 centred (before target_correction_mm), 3 the TCP reached. 2 - 1 large: the camera model
    is off; 3 - 2: what the robot did not reach. TCP visibly off the real stud although
    3 - 2 is small: constant camera -> TCP offset (target_correction_mm)."""
    observed = np.asarray(found["observed_mm"][:2])
    centred = np.asarray(found["centred_mm"][:2])
    tcp = np.asarray(reached_tcp[:2])
    for label, xy in (("1 observation pose", observed), ("2 centred", centred),
                      ("3 TCP reached", tcp), ("2 - 1 (camera model)", centred - observed),
                      ("3 - 2 (robot)", tcp - centred)):
        log.info("check %-22s x %+8.2f  y %+8.2f mm", label, *xy)
    return {"centred_minus_observed_mm": rounded(centred - observed),
            "tcp_minus_centred_mm": rounded(tcp - centred)}


def place_and_screw(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
                    poses: config.Poses, record: dict, approach_only: bool = False) -> None:
    """Step 2: observation pose over the stud -> stud search -> one move to just above the
    thread -> descend until contact -> screw (compliant) -> release -> back up.
    approach_only: stop above the thread tip and hold (centring check, no gripper)."""
    table_z = poses.table_z()
    stud = cfg["objects"]["stud"]
    place = cfg["place"]
    modes = cfg["robot"]["modes"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    observe = observation_pose(poses, "observe_stud", cfg)
    robot.set_mode(modes["normal"])
    vision.prepare(TARGET_STUD)
    move_to(robot, observe, cfg)
    found = record["stud"] = find_stud(robot, vision, cfg, table_z, observe, record)

    # The nut sits in the iris with its bottom face at the TCP plane
    tip_z = table_z + stud["top_height_mm"]
    approach = frames.flange_for_tcp([*found["position_mm"][:2],
                                      tip_z + place["approach_clearance_mm"]],
                                     config.rotation_down(cfg), tcp_offset)
    move_to(robot, approach, cfg, lead=LEAD_XY)
    if approach_only:
        reached = frames.tcp_position(robot.flange_pose(), tcp_offset)
        log.info("approach only: TCP %.1f mm above the thread tip; holding here",
                 reached[2] - tip_z)
        record["approach_reached_tcp_mm"] = rounded(reached)
        record["check"] = position_check(found, reached)
        return

    descent = straight_down(approach, place["max_descend_mm"], place["descend_speed_mm_s"],
                            cfg["robot"]["stream_rate_hz"])
    record["contact"] = robot.follow(descent, stop_on_contact_n=place["contact_force_n"])
    if not record["contact"]:
        if not place["screw_without_contact"]:
            move_to(robot, approach, cfg, lead=LEAD_Z)
            raise SequenceError(f"no contact within {place['max_descend_mm']} mm: nut not on "
                                "the thread?")
        log.warning("no contact within %.0f mm: screwing at the lowest point anyway",
                    place["max_descend_mm"])
    contact = robot.flange_pose()
    record["contact_tcp_mm"] = rounded(frames.tcp_position(contact, tcp_offset))
    screw = place["screw"]
    robot.set_mode(modes["float"])
    below = contact.copy()
    below[2, 3] -= screw["follow_depth_mm"]
    robot.follow([Waypoint(0.0, below)])
    gripper.rotate(screw["direction"], screw["duration_ms"], screw["speed"])
    gripper.open()
    # Reference back to where the arm is before the stiff mode returns, else it would jump
    # towards the reference below the thread
    robot.follow([Waypoint(0.0, robot.flange_pose())])
    robot.set_mode(modes["normal"])
    move_to(robot, observe, cfg, lead=LEAD_Z)
