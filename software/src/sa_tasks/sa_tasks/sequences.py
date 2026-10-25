"""Step 1 (pick the nut) and step 2 (place it on the stud and screw it on). No ROS.

docs/PLAN.md Section 4, docs/THEORY.md Sections 2, 3 and 5. Only the Robot, Gripper and
Vision interfaces are used, so the sequences run against mocks.
"""
import logging
import threading
import time
from typing import Callable, Optional

import numpy as np

from sa_tasks import config, frames
from sa_tasks.interfaces import Gripper, Robot, Vision
from sa_tasks.trajectory import (LEAD_XY, LEAD_Z, Waypoint, plan_move, spiral_down,
                                 straight_down)

log = logging.getLogger("sa_tasks")

TARGET_NUT = "nut"
TARGET_STUD = "stud"


class SequenceError(RuntimeError):
    """A step failed; the robot stays where it is (no automatic recovery)."""


def rounded(values) -> list:
    return [round(float(v), 2) for v in values]


def stopwatch(record: dict) -> Callable[[str], None]:
    """mark(label): record["time_s"][label] = seconds since this call (where a run spends
    its time)."""
    started, times = time.monotonic(), record.setdefault("time_s", {})

    def mark(label: str) -> None:
        times[label] = round(time.monotonic() - started, 2)
    return mark


def move_to(robot: Robot, goal: np.ndarray, cfg: dict, lead: Optional[str] = None,
            precise: bool = False) -> np.ndarray:
    """One continuous move of the flange from where it is to goal, then wait for arrival.
    precise: then shift the reference by the remaining xy error of the TCP (from the measured
    flange pose, so a tilt of the arm counts too) until it is within
    fine_positioning.tolerance_mm (THEORY 5). Returns the last reference sent."""
    limits, rate = config.motion_limits(cfg), cfg["robot"]["stream_rate_hz"]
    tolerance = cfg["robot"]["arrival_tolerance_mm"]
    robot.follow(plan_move(robot.flange_pose(), goal, limits, rate, lead))
    robot.wait_until_arrived(goal, tolerance)
    reference = goal
    if not precise:
        return reference
    fine = cfg["robot"]["fine_positioning"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    target = frames.tcp_position(goal, tcp_offset)[:2]

    def tcp_error() -> np.ndarray:
        return frames.tcp_position(robot.flange_pose(), tcp_offset)[:2] - target
    for _ in range(fine["max_steps"]):
        error = tcp_error()
        if np.linalg.norm(error) <= fine["tolerance_mm"]:
            break
        corrected = reference.copy()
        corrected[:2, 3] -= fine["gain"] * error
        robot.follow(plan_move(reference, corrected, limits, rate))
        robot.wait_until_arrived(corrected, tolerance)
        reference = corrected
    log.info("fine positioning: TCP xy error x %+.1f y %+.1f mm, reference shifted by "
             "x %+.1f y %+.1f mm", *tcp_error(), *(reference[:2, 3] - goal[:2, 3]))
    return reference


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
    """Step 1: observation pose (the iris opens on the way) -> detect the nut -> move to just
    above it and on without stopping -> spiral search down until the bottom plate rests on the
    table, the open iris over the nut -> close -> back up. back_up False (full run): stay on the table, the next move
    (to observe_stud, z leading) lifts the nut in one motion."""
    mark = stopwatch(record)
    table_z = poses.table_z()
    observe = observation_pose(poses, "observe_nut", cfg)
    robot.set_mode(cfg["robot"]["modes"]["normal"])
    vision.prepare(TARGET_NUT)
    move_while(robot, observe, cfg, gripper.open)
    mark("at_observe_nut")
    nut = record["nut_mm"] = locate_nut(robot, vision, cfg, table_z)
    mark("nut_located")

    # Down to just above the nut and straight on (no stop, no fine positioning there; 23.10.)
    # into a spiral down onto the table: if the target is off by up to the spiral radius, the
    # plate slides over the nut until the open iris passes over it and the nut drops in (THEORY 5)
    spiral = cfg["pick"]["spiral"]
    press = cfg["pick"]["press_depth_mm"]
    start = frames.flange_for_tcp([nut[0], nut[1], table_z + spiral["start_height_mm"]],
                                  config.rotation_down(cfg), cfg["frames"]["flange_to_tcp_mm"])
    robot.follow(plan_move(robot.flange_pose(), start, config.motion_limits(cfg),
                           cfg["robot"]["stream_rate_hz"], LEAD_XY))
    search = spiral_down(start, spiral["start_height_mm"] + press, spiral["radius_mm"],
                         spiral["turns"], spiral["duration_s"], cfg["robot"]["stream_rate_hz"])
    robot.follow(search)
    # If the iris misses, the plate stands on the nut instead of the table: accept that
    robot.wait_until_arrived(search[-1].t_base_flange, cfg["robot"]["arrival_tolerance_mm"]
                             + press + cfg["objects"]["nut"]["height_mm"])
    landed = frames.tcp_position(robot.flange_pose(), cfg["frames"]["flange_to_tcp_mm"])
    record["landed_tcp_mm"] = rounded(landed)
    log.info("landed: TCP %.1f mm above the table", landed[2] - table_z)
    mark("landed")
    gripper.close()
    mark("closed")
    if back_up:
        move_to(robot, observe, cfg, lead=LEAD_Z)
        mark("back")


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


def stiffen_where_it_is(robot: Robot, cfg: dict) -> np.ndarray:
    """From the soft screw mode to the stiff mode without a drop or jump, before the iris lets
    go: reference to the measured pose plus release_lift_mm (the weight the stiff mode does not
    compensate, no z integrator), wait in the soft mode until the controller's low-passed
    reference (tau ~0.1 s) has come up from below the thread, then stiff. Returns the
    reference. Opening first let the soft arm fall onto the reference below the thread."""
    place = cfg["place"]
    hold = robot.flange_pose()
    hold[2, 3] += place["release_lift_mm"]
    robot.follow([Waypoint(0.0, hold)])
    time.sleep(place["release_settle_s"])
    robot.set_mode(cfg["robot"]["modes"]["normal"])
    return hold


def screw_on(robot: Robot, gripper: Gripper, cfg: dict, contact: np.ndarray,
             free_fz: float, record: dict) -> None:
    """Screw the nut on from the contact pose in the screw mode (soft tilt, no integral
    action: the thread can align the nut). Turn slowly in short steps, the reference moving one
    point further on a small circle around the contact each step (wobble, to find the start of
    the thread), until the TCP has sunk by caught_drop_mm (the thread has caught); then centred
    again, faster and with more preload.
    The reference is depth_mm below contact, i.e. a preload of about k_z * depth_mm plus what
    the controller does not compensate; the record holds the measured Fz - free_fz."""
    screw = cfg["place"]["screw"]
    catch, tighten = screw["catch"], screw["tighten"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]

    contact_tcp = frames.tcp_position(contact, tcp_offset)

    def state() -> tuple:  # TCP z, tilt of the flange, contact force estimate (plain floats)
        flange = robot.flange_pose()
        return (float(frames.tcp_position(flange, tcp_offset)[2]),
                frames.tilt_deg(flange[:3, :3]), round(float(robot.force_z() - free_fz), 2))

    def tcp_xy_from_contact() -> list:  # did the wobble move the arm?
        return rounded(frames.tcp_position(robot.flange_pose(), tcp_offset)[:2] - contact_tcp[:2])

    robot.set_mode(cfg["robot"]["modes"]["screw"])
    below = contact.copy()
    below[2, 3] = contact[2, 3] - catch["depth_mm"]
    robot.follow([Waypoint(0.0, below)])
    time.sleep(catch["settle_s"])
    start_z, tilt, force = state()
    record.update(tilt_start_deg=round(tilt, 2), force_start_n=force, catch_drops_mm=[],
                  catch_tcp_xy_from_contact_mm=[], caught=False)
    wobble = catch["wobble"]
    for step in range(catch["max_steps"]):
        angle = 2.0 * np.pi * step / wobble["points_per_circle"]
        below[:2, 3] = contact[:2, 3] + wobble["radius_mm"] * np.array([np.cos(angle),
                                                                         np.sin(angle)])
        robot.follow([Waypoint(0.0, below)])
        gripper.rotate(screw["direction"], catch["step_ms"], catch["speed"])
        z, tilt, force = state()
        record["catch_drops_mm"].append(round(start_z - z, 2))
        record["catch_tcp_xy_from_contact_mm"].append(tcp_xy_from_contact())
        if start_z - z >= catch["caught_drop_mm"]:
            record["caught"] = True
            break
    record.update(tilt_after_catch_deg=round(tilt, 2), force_after_catch_n=force)
    if record["caught"]:
        log.info("thread caught after %d steps (%.2f mm down)", len(record["catch_drops_mm"]),
                 start_z - z)
    else:
        log.warning("thread not caught after %d steps (%.2f mm down, tilt %.1f deg): "
                    "tightening anyway", catch["max_steps"], start_z - z, tilt)
    below[:3, 3] = contact[:3, 3] - [0.0, 0.0, tighten["depth_mm"]]  # centred again
    robot.follow([Waypoint(0.0, below)])
    robot.set_mode(cfg["robot"]["modes"]["tighten"])  # soft xy: the nut may swing while turning
    gripper.rotate(screw["direction"], tighten["duration_ms"], tighten["speed"])
    z, tilt, force = state()
    record.update(drop_end_mm=round(start_z - z, 2), tilt_end_deg=round(tilt, 2),
                  force_end_n=force)


def place_and_screw(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
                    poses: config.Poses, record: dict, approach_only: bool = False) -> None:
    """Step 2: observation pose over the stud -> stud search -> one move to just above the
    thread -> descend until contact -> screw (screw_on) -> release -> back up.
    approach_only: stop above the thread tip and hold (centring check, no gripper)."""
    mark = stopwatch(record)
    table_z = poses.table_z()
    stud = cfg["objects"]["stud"]
    place = cfg["place"]
    modes = cfg["robot"]["modes"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    observe = observation_pose(poses, "observe_stud", cfg)
    robot.set_mode(modes["normal"])
    vision.prepare(TARGET_STUD)
    move_to(robot, observe, cfg)
    mark("at_observe_stud")
    found = record["stud"] = find_stud(robot, vision, cfg, table_z, observe, record)
    mark("stud_found")

    # The nut sits in the iris with its bottom face at the TCP plane
    tip_z = table_z + stud["top_height_mm"]
    approach = frames.flange_for_tcp([*found["position_mm"][:2],
                                      tip_z + place["approach_clearance_mm"]],
                                     config.rotation_down(cfg), tcp_offset)
    approach = move_to(robot, approach, cfg, lead=LEAD_XY, precise=True)
    mark("approached")
    if approach_only:
        reached = frames.tcp_position(robot.flange_pose(), tcp_offset)
        log.info("approach only: TCP %.1f mm above the thread tip; holding here",
                 reached[2] - tip_z)
        record["approach_reached_tcp_mm"] = rounded(reached)
        record["check"] = position_check(found, reached)
        return

    free_fz = robot.force_z()  # in the air: the estimate's bias, so Fz - free_fz = contact force
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
    mark("contact")
    record["contact_tcp_mm"] = rounded(frames.tcp_position(contact, tcp_offset))
    record["screw"] = {}
    screw_on(robot, gripper, cfg, contact, free_fz, record["screw"])
    mark("screwed")
    stiffen_where_it_is(robot, cfg)
    gripper.open()
    mark("released")
    move_to(robot, observe, cfg, lead=LEAD_Z)
    mark("back")
