"""Step 1 (pick the nut) and step 2 (place it on the stud and screw it on). No ROS.

docs/PLAN.md Section 4, docs/THEORY.md Sections 2, 3 and 5. Only the Robot, Gripper and
Vision interfaces are used, so the sequences run against mocks.
"""
import dataclasses
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


def no_mark(_event: str) -> None:
    """Default of the mark callbacks: time marks of a run (run_record.Timeline) not wanted."""


class SequenceError(RuntimeError):
    """A step failed; the robot stays where it is (no automatic recovery)."""


def rounded(values) -> list:
    return [round(float(v), 2) for v in values]


def move_to(robot: Robot, goal: np.ndarray, cfg: dict, lead: Optional[str] = None,
            precise: bool = False, rough: bool = False,
            lead_fraction: Optional[float] = None,
            max_speed_z_mm_s: Optional[float] = None) -> np.ndarray:
    """One continuous move of the flange from where it is to goal, then wait for arrival.
    rough: arrived already while creeping (robot.rough_arrival), for a first look only.
    lead_fraction, max_speed_z_mm_s: instead of robot.motion's values for this move.
    precise: then shift the reference by the remaining xy error of the TCP (from the measured
    flange pose, so a tilt of the arm counts too) until it is within
    fine_positioning.tolerance_mm (THEORY 5). Returns the last reference sent."""
    limits, rate = config.motion_limits(cfg), cfg["robot"]["stream_rate_hz"]
    if lead_fraction is not None:
        limits = dataclasses.replace(limits, lead_fraction=lead_fraction)
    if max_speed_z_mm_s is not None:
        limits = dataclasses.replace(limits, max_speed_z_mm_s=max_speed_z_mm_s)
    tolerance = cfg["robot"]["arrival_tolerance_mm"]
    robot.follow(plan_move(robot.flange_pose(), goal, limits, rate, lead))
    # precise: the reference is re-sent while waiting, so the controller's integral term stays
    # zero and the error is measured and corrected as the next streamed path will see it
    if rough:
        r = cfg["robot"]["rough_arrival"]
        robot.wait_until_arrived(goal, r["tolerance_mm"], settle_s=r["settle_s"],
                                 speed_mm_s=r["speed_mm_s"])
    else:
        robot.wait_until_arrived(goal, tolerance, hold=goal if precise else None)
    reference = goal
    if not precise:
        return reference
    fine = cfg["robot"]["fine_positioning"]
    # the correction moves are a few mm: their own shortest duration (27.10.: 1.0 s as every move)
    step_limits = dataclasses.replace(limits, min_duration_s=fine["min_duration_s"])
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
        robot.follow(plan_move(reference, corrected, step_limits, rate))
        robot.wait_until_arrived(corrected, tolerance, hold=corrected)
        reference = corrected
    log.info("fine positioning: TCP xy error x %+.1f y %+.1f mm, reference shifted by "
             "x %+.1f y %+.1f mm", *tcp_error(), *(reference[:2, 3] - goal[:2, 3]))
    return reference


def alongside(action: Callable[[], None], main: Callable[[], None]) -> None:
    """main() while action() runs in a second thread; returns when both are done, errors of
    both raise."""
    errors = []

    def run():
        try:
            action()
        except Exception as error:  # noqa: BLE001 - re-raised below
            errors.append(error)
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    main()
    worker.join()
    if errors:
        raise errors[0]


def move_while(robot: Robot, goal: np.ndarray, cfg: dict, action: Callable[[], None],
               rough: bool = False) -> None:
    """move_to while action() runs alongside (e.g. the iris opening); errors of both raise."""
    alongside(action, lambda: move_to(robot, goal, cfg, rough=rough))


def observation_pose(poses: config.Poses, name: str, cfg: dict) -> np.ndarray:
    """Taught position with the fixed orientation. With robot.observation_height_from set,
    z comes from that pose (both observation poses share one height, same image scale)."""
    position = poses.get(name)[:3, 3].copy()
    if cfg["robot"]["observation_height_from"]:
        position[2] = poses.get(cfg["robot"]["observation_height_from"])[2, 3]
    return frames.pose(config.rotation_down(cfg), position)


def target_correction(cfg: dict, xy_mm) -> np.ndarray:
    """xy correction of a target at xy_mm (base frame): per component a plane
    c0 + cx * x + cy * y, frames.target_correction (27.10.: measured with tcp_offset in the four
    corners of the table, the x component changes from one side of the table to the other)."""
    planes = cfg["frames"]["target_correction"]
    x, y = float(xy_mm[0]), float(xy_mm[1])
    return np.array([c0 + cx * x + cy * y for c0, cx, cy in (planes["x"], planes["y"])])


def camera_and_distance(robot: Robot, cfg: dict, plane_z_mm: float) -> tuple:
    t_base_camera = robot.flange_pose() @ config.camera_in_flange(cfg)
    return t_base_camera, frames.distance_along_axis(t_base_camera, plane_z_mm)


def camera_during(robot: Robot, cfg: dict, before: np.ndarray) -> np.ndarray:
    """Camera pose for a view taken while the arm still crept (rough): the flange position
    halfway between before the request and now, i.e. about when the images of the median were
    taken (27.10.)."""
    flange = robot.flange_pose()
    flange[:3, 3] = (flange[:3, 3] + before[:3, 3]) / 2.0
    camera = flange @ config.camera_in_flange(cfg)
    log.info("view while creeping: arm moved %.1f mm during the images",
             float(np.linalg.norm(robot.flange_pose()[:3, 3] - before[:3, 3])))
    return camera


def close_up_pose(cfg: dict, point_xy, plane_z_mm: float) -> np.ndarray:
    """Flange pose (fixed orientation) whose optical axis passes through point_xy on the plane
    z = plane_z_mm at vision.close_up_distance_mm from the projection centre (26.10.: every
    angle error of the camera model or of the arm moves the result by distance * angle, so
    the second measurement is taken from close by; THEORY 3.1)."""
    rotation = config.rotation_down(cfg)
    t_flange_camera = config.camera_in_flange(cfg)
    axis = rotation @ t_flange_camera[:3, 2]  # z_C in the base frame (points down)
    point = np.array([point_xy[0], point_xy[1], plane_z_mm])
    centre = point - cfg["vision"]["close_up_distance_mm"] * axis
    return frames.pose(rotation, centre - rotation @ t_flange_camera[:3, 3])


def locate_nut(robot: Robot, vision: Vision, cfg: dict, table_z: float,
               rough: bool = False) -> list:
    """Nut position in the base frame (incl. target_correction) from the current pose.
    rough: first look, the arm may still creep (camera_during)."""
    nut = cfg["objects"]["nut"]
    before = robot.flange_pose()
    camera, distance = camera_and_distance(robot, cfg, table_z + nut["height_mm"])
    seen = vision.observe(TARGET_NUT, distance, rough)
    if rough:
        camera = camera_during(robot, cfg, before)
    plane_z = table_z + nut["centre_height_mm"]
    point = frames.object_in_base(seen.u_undistorted, seen.v_undistorted, seen.k, camera, plane_z)
    axis = frames.object_in_base(seen.k[0, 2], seen.k[1, 2], seen.k, camera, plane_z)
    offset = float(np.linalg.norm(point[:2] - axis[:2]))
    if offset > cfg["vision"]["max_offset_from_axis_mm"]:
        raise SequenceError(f"nut found {offset:.0f} mm from the camera axis, outside the "
                            "work area")
    point[:2] += target_correction(cfg, point[:2])
    log.info("nut at x %.1f y %.1f mm, %.0f mm from the camera axis", point[0], point[1], offset)
    return rounded(point)


def stud_view(robot: Robot, vision: Vision, cfg: dict, table_z: float, lock: bool,
              rough: bool = False) -> tuple:
    """All objects of one view (rank order of vision_node) on the stud's centre plane.
    Returns (camera axis point on that plane, [dict per object]). rough: as locate_nut."""
    stud = cfg["objects"]["stud"]
    before = robot.flange_pose()
    camera, distance = camera_and_distance(robot, cfg, table_z + stud["base_height_mm"])
    view = vision.view(TARGET_STUD, distance, lock, rough)
    if rough:
        camera = camera_during(robot, cfg, before)
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


def find_stud(robot: Robot, vision: Vision, cfg: dict, table_z: float, record: dict,
              mark: Callable[[str], object] = no_mark) -> dict:
    """Stud search (THEORY 2.0): the tall stud is a clean square only from directly above.
    The objects locked at the observation pose are tried in rank order: the camera axis is put
    over the object from close by (close_up_pose, vision.close_up_distance_mm; THEORY 5), and
    the largest object within lost_radius_mm of the axis is judged there (dark spots on the
    stud can be separate small blobs). Is it more than centred_tolerance_mm off the axis (the
    first estimate was far off), the camera moves over it again. The first square is the
    stud; its position comes from that close view, where the parallax of the tall stud is
    negligible. 27.10. (Yannick): until then the camera was first centred at the observation
    height and only then moved down for the close-up measurement, one move and one detection
    more."""
    search = cfg["place"]["stud_search"]
    plane_z = table_z + cfg["objects"]["stud"]["base_height_mm"]
    _, seen = stud_view(robot, vision, cfg, table_z, lock=True, rough=True)
    mark("stud_observed")
    record["candidates"] = seen
    candidates = [c for c in seen if c["offset_from_axis_mm"]
                  <= cfg["vision"]["max_offset_from_axis_mm"]][:search["max_candidates"]]
    if not candidates:
        raise SequenceError(f"no object in the work area at the stud observation pose "
                            f"({len(seen)} in view)")
    attempts = record["attempts"] = []
    for candidate in candidates:
        position = np.asarray(candidate["position_mm"][:2])
        for move in range(1, search["max_centring_moves"] + 1):
            move_to(robot, close_up_pose(cfg, position, plane_z), cfg)
            mark("stud_close_arrived")
            _, now = stud_view(robot, vision, cfg, table_z, lock=False)
            near = [c for c in now if c["offset_from_axis_mm"] <= search["lost_radius_mm"]]
            nearest = max(near, key=lambda c: c["size_mm"]) if near else None
            attempts.append({"candidate": candidate["rank"], "move": move, "nearest": nearest,
                             "seen": now})
            if nearest is None:
                log.info("candidate #%d lost after close-up move %d: %d object(s) in view, "
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
            target[:2] += target_correction(cfg, target[:2])
            moved = position - np.asarray(candidate["position_mm"][:2])
            log.info("stud = candidate #%d (w %.2f, %.1f mm) at x %.1f y %.1f mm after %d "
                     "move(s), %+.2f %+.2f mm from the observation pose", candidate["rank"],
                     nearest["width_ratio"], nearest["size_mm"], target[0], target[1], move,
                     *moved)
            return {"position_mm": rounded(target), "close_mm": nearest["position_mm"],
                    "observed_mm": candidate["position_mm"], "candidate": candidate["rank"],
                    "moves": move, "close_minus_observed_mm": rounded(moved)}
    raise SequenceError(f"none of the {len(candidates)} object(s) tried is a stud")


def pick(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict, poses: config.Poses,
         record: dict, back_up: bool = True, mark: Callable[[str], object] = no_mark) -> None:
    """Step 1: observation pose (the iris opens on the way) -> detect the nut -> camera over it
    from close by, detect again -> move to just above it -> straight down until the bottom
    plate rests on the table, the open iris over the nut -> close -> back up. back_up False
    (full run): stay on the table, the next move (to observe_stud, z leading) lifts the nut in
    one motion. mark(event) at the steps (run_record.Timeline)."""
    table_z = poses.table_z()
    observe = observation_pose(poses, "observe_nut", cfg)
    robot.set_mode(cfg["robot"]["modes"]["normal"])
    vision.prepare(TARGET_NUT)
    # First look while the arm still creeps (27.10., Yannick): the close-up corrects it
    move_while(robot, observe, cfg, gripper.open, rough=True)
    mark("observe_nut")
    coarse = record["nut_coarse_mm"] = locate_nut(robot, vision, cfg, table_z, rough=True)
    mark("nut_observed")
    move_to(robot, close_up_pose(cfg, coarse[:2], table_z + cfg["objects"]["nut"]["height_mm"]),
            cfg)
    mark("nut_close_arrived")
    nut = record["nut_mm"] = locate_nut(robot, vision, cfg, table_z)
    record["nut_close_minus_coarse_mm"] = rounded(np.subtract(nut[:2], coarse[:2]))
    log.info("nut close up: %+.2f %+.2f mm from the first measurement",
             *record["nut_close_minus_coarse_mm"])
    mark("nut_located")

    # One move from the close-up pose to below the table straight under the nut (xy leading, so
    # the end is vertical) until the plate pushes on the table. Contact counts only once the
    # reference is below contact_height_mm. 27.10. (Yannick): the stop above the nut and the slow
    # descent at 10 mm/s dropped. A plain move without contact detection stopped ~5 mm above
    # the table (26.10.: joint friction, standing still counted as landed). The iris misses:
    # the plate stands on the nut, also contact. The force baseline is taken at the close-up
    # pose, where the arm stands still after the detection.
    descent = cfg["pick"]["descent"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    rotation = config.rotation_down(cfg)
    goal = frames.flange_for_tcp([nut[0], nut[1], table_z - descent["below_table_mm"]],
                                 rotation, tcp_offset)
    path = plan_move(robot.flange_pose(), goal, config.motion_limits(cfg),
                     cfg["robot"]["stream_rate_hz"], LEAD_XY)
    low = table_z + descent["contact_height_mm"]
    contact_from = next(w.time_s for w in path
                        if frames.tcp_position(w.t_base_flange, tcp_offset)[2] <= low)
    record["contact"] = robot.follow(path, stop_on_contact_n=descent["contact_force_n"],
                                     contact_from_s=contact_from)
    if record["contact"]:  # plate flat on the table with a little preload while the iris closes
        pressed = robot.flange_pose()
        pressed[:3, :3] = config.rotation_down(cfg)
        pressed[2, 3] -= cfg["pick"]["press_depth_mm"]
        robot.follow([Waypoint(0.0, pressed)])
    else:  # usual since the one-move pick: the arm trails the fast reference by a few cm and
        # only reaches the table after the path (reference 8 mm below the table) has ended
        log.info("no contact during the move: the arm comes down onto the table by itself")
    # the plate settles flat on the table before the iris closes (27.10.: since the one-move
    # pick it arrives faster and the iris closed while the plate still tilted)
    time.sleep(cfg["pick"]["flush_settle_s"])
    mark("landed")
    # measured after the wait, with the plate on the table (27.10.: at the end of the path the
    # TCP was still 32 mm up in the air)
    landed = frames.tcp_position(robot.flange_pose(), tcp_offset)
    record["landed_tcp_mm"] = rounded(landed)
    # Computed target vs. where the iris came down (the robot's share of a missed nut)
    record["landed_minus_target_xy_mm"] = rounded(landed[:2] - np.asarray(nut[:2]))
    log.info("landed: TCP %.1f mm above the table", landed[2] - table_z)
    gripper.close()
    mark("iris_closed")
    if back_up:
        move_to(robot, observe, cfg, lead=LEAD_Z)


def position_check(found: dict, reached_tcp: np.ndarray) -> dict:
    """Stud xy at three stages (approach_only): 1 from the observation pose (off-axis),
    2 from close by (before target_correction), 3 the TCP reached. 2 - 1 large: the camera
    model is off; 3 - 2: what the robot did not reach plus target_correction. TCP visibly off
    the real stud: camera -> TCP offset (target_correction, tcp_offset)."""
    observed = np.asarray(found["observed_mm"][:2])
    close = np.asarray(found["close_mm"][:2])
    tcp = np.asarray(reached_tcp[:2])
    for label, xy in (("1 observation pose", observed), ("2 close up", close),
                      ("3 TCP reached", tcp), ("2 - 1 (camera model)", close - observed),
                      ("3 - 2 (robot)", tcp - close)):
        log.info("check %-22s x %+8.2f  y %+8.2f mm", label, *xy)
    return {"close_minus_observed_mm": rounded(close - observed),
            "tcp_minus_close_mm": rounded(tcp - close)}


def stiffen_where_it_is(robot: Robot, cfg: dict) -> np.ndarray:
    """From the soft screw mode to the stiff mode without a drop or jump, before the iris lets
    go: reference to the measured pose plus release_lift_mm (the weight the stiff mode does not
    compensate, no z integrator), wait in the soft mode until the controller's low-passed
    reference (tau ~0.5 s) has come up from below the thread, then stiff. Returns the
    reference. Opening first let the soft arm fall onto the reference below the thread."""
    place = cfg["place"]
    hold = robot.flange_pose()
    hold[2, 3] += place["release_lift_mm"]
    robot.follow([Waypoint(0.0, hold)])
    time.sleep(place["release_settle_s"])
    robot.set_mode(cfg["robot"]["modes"]["normal"])
    return hold


def seat_nut(robot: Robot, cfg: dict, contact: np.ndarray, free_fz: float,
             record: dict) -> tuple:
    """Before screwing: press down on the stud in the stiff mode, so the nut slides up in the
    iris against its top stop (26.10., Yannick: the thread usually catches, the fast turn
    never ran on). Reference depth_mm below contact, wait settle_s (reference low-pass), then
    the measured pose is the new contact. The record holds how far the TCP went down (the nut
    moving up in the iris, the stud being rigid) and the force. Returns (measured pose,
    z of the reference)."""
    seat = cfg["place"]["seat"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    pressed = contact.copy()
    pressed[2, 3] -= seat["depth_mm"]
    robot.follow([Waypoint(0.0, pressed)])
    time.sleep(seat["settle_s"])
    seated = robot.flange_pose()
    seated_tcp = frames.tcp_position(seated, tcp_offset)
    travel = float(frames.tcp_position(contact, tcp_offset)[2] - seated_tcp[2])
    record.update(seated_tcp_mm=rounded(seated_tcp), travel_mm=round(travel, 2),
                  force_n=round(float(robot.force_z() - free_fz), 2))
    log.info("seat: TCP %.2f mm down, force %.1f N", travel, record["force_n"])
    return seated, float(pressed[2, 3])


def find_thread(robot: Robot, cfg: dict, contact: np.ndarray, record: dict,
                ceiling_z: float = np.inf) -> tuple:
    """Spiral search on the thread tip (26.10.): the stud target is off by 1-3 mm, so the nut
    rests on the rim of the tip. Pressed lightly onto it, the arm runs a flat outward spiral
    around the contact while the TCP height is sampled; where the nut is over the thread it
    drops onto the thread start (chamfers of nut and tip), so the lowest TCP marks the centre.
    The arm then goes back to that xy, and the measured pose there is the new contact.
    27.10. (Yannick): the spiral stops as soon as the TCP has sunk found_drop_mm below the
    start (the nut dropped onto the thread start), and the screwing starts there at once.
    The reference stays at or below ceiling_z (the reference before, 27.10.: else the arm rose
    as the pressure dropped). Returns (measured pose, z of the reference)."""
    search = cfg["place"]["thread_search"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    start = contact.copy()
    start[2, 3] = min(contact[2, 3] - search["press_mm"], ceiling_z)
    robot.follow([Waypoint(0.0, start)])
    time.sleep(search["settle_s"])
    samples = [robot.flange_pose()]
    forces, times, began = [robot.force_z()], [0.0], time.monotonic()
    done = threading.Event()

    def sample():
        while not done.is_set():
            samples.append(robot.flange_pose())
            forces.append(robot.force_z())
            times.append(time.monotonic() - began)
            time.sleep(search["sample_period_s"])
    worker = threading.Thread(target=sample, daemon=True)
    worker.start()
    start_z = frames.tcp_position(samples[0], tcp_offset)[2]
    found = robot.follow(
        spiral_down(start, 0.0, search["radius_mm"], search["turns"], search["duration_s"],
                    cfg["robot"]["stream_rate_hz"]),
        stop_when=lambda: (start_z - frames.tcp_position(robot.flange_pose(), tcp_offset)[2]
                           >= search["found_drop_mm"]))
    if not found:
        time.sleep(search["settle_s"])  # the low-passed reference still runs on
    done.set()
    worker.join()
    samples.append(robot.flange_pose())
    forces.append(robot.force_z())
    times.append(time.monotonic() - began)

    tcp = np.array([frames.tcp_position(s, tcp_offset) for s in samples])
    # 27.10.: the curves for choosing the stop signal (TCP height below the start, Fz minus
    # its value at the start), every trace_every-th sample
    every = search["trace_every"]
    record["trace"] = {"t_s": [round(t, 2) for t in times[::every]],
                       "drop_mm": [round(float(tcp[0, 2] - z), 3) for z in tcp[::every, 2]],
                       "fz_n": [round(float(f - forces[0]), 2) for f in forces[::every]]}
    # found: where the nut dropped, i.e. where the arm is now (the last sample)
    lowest = len(samples) - 1 if found else int(np.argmin(tcp[:, 2]))
    best = start.copy()
    best[:2, 3] = samples[lowest][:2, 3]
    robot.follow([Waypoint(0.0, best)])
    if not found:
        time.sleep(search["settle_s"])  # back to the lowest point first
    contact_tcp = frames.tcp_position(contact, tcp_offset)
    record.update(found=found, 
        lowest_tcp_mm=rounded(tcp[lowest]),
        drop_mm=round(float(tcp[0, 2] - tcp[lowest, 2]), 2),  # below the pressed start
        shift_xy_mm=rounded(samples[lowest][:2, 3] - contact[:2, 3]),
        reached_radius_mm=round(float(np.max(np.linalg.norm(tcp[:, :2] - contact_tcp[:2],
                                                             axis=1))), 2),
        samples=len(samples))
    log.info("thread search: %s, %.2f mm below the start, %+.2f %+.2f mm from it (spiral "
             "reached %.1f mm)", "dropped onto the thread" if found else "lowest point",
             record["drop_mm"], *record["shift_xy_mm"], record["reached_radius_mm"])
    return robot.flange_pose(), float(best[2, 3])


def screw_on(robot: Robot, gripper: Gripper, cfg: dict, contact: np.ndarray,
             free_fz: float, record: dict, mark: Callable[[str], object] = no_mark,
             ceiling_z: float = np.inf) -> None:
    """Screw the nut on from the contact pose in the screw mode (soft tilt, no integral
    action: the thread can align the nut). Turn slowly in short steps, the reference moving one
    point further on a small circle around the contact each step (wobble, to find the start of
    the thread), until the TCP has sunk by caught_drop_mm (the thread has caught); then centred
    again, faster and with more preload.
    The reference is depth_mm below contact, i.e. a preload of about k_z * depth_mm plus what
    the controller does not compensate; the record holds the measured Fz - free_fz.
    The catch reference never goes above ceiling_z (the reference of the thread search; 27.10.,
    Yannick: the arm rose when the pressure dropped). The tightening starts depth_mm below the
    measured pose instead (27.10. round 2: at the search depth it pressed with 6-7 N)."""
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
    below[2, 3] = min(contact[2, 3] - catch["depth_mm"], ceiling_z)
    robot.follow([Waypoint(0.0, below)])
    if catch["max_steps"]:  # 0: no catch phase, straight to tightening (27.10.)
        time.sleep(catch["settle_s"])
    start_z, tilt, force = state()
    z = start_z
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
    mark("catch_done")
    if record["caught"]:
        log.info("thread caught after %d steps (%.2f mm down)", len(record["catch_drops_mm"]),
                 start_z - z)
    elif catch["max_steps"]:
        log.warning("thread not caught after %d steps (%.2f mm down, tilt %.1f deg): "
                    "tightening anyway", catch["max_steps"], start_z - z, tilt)
    below[:2, 3] = contact[:2, 3]  # centred again
    # depth_mm below the measured pose, not the deeper reference of the search: 27.10. round 2
    # pressed with 6-7 N at the start (search reference ~6 mm below, tighten z 600 N/m) and the
    # nut jammed on the left. The nut rests on the stud, so the arm cannot rise from this
    below[2, 3] = robot.flange_pose()[2, 3] - tighten["depth_mm"]
    robot.follow([Waypoint(0.0, below)])
    robot.set_mode(cfg["robot"]["modes"]["tighten"])  # soft xy: the nut may swing while turning
    # While M4 turns, the reference is kept depth_mm below the measured pose (every
    # follow_period_s): the gripper follows the nut down the thread with a constant light
    # preload (k_z * depth_mm). 27.10.: a reference running down at a fixed speed raised the
    # preload when the nut stalled (left side: 1.5-5 N, the M4 could not turn the nut)
    turning = threading.Event()

    def follow_nut():
        while not turning.wait(tighten["follow_period_s"]):
            reference = below.copy()
            reference[2, 3] = robot.flange_pose()[2, 3] - tighten["depth_mm"]
            robot.follow([Waypoint(0.0, reference)])

    def turn():
        try:
            gripper.rotate(screw["direction"], tighten["duration_ms"], tighten["speed"])
        finally:
            turning.set()  # also on an error, else follow_nut never ends
    alongside(turn, follow_nut)
    mark("m4_stopped")
    z, tilt, force = state()
    # end_tcp_mm: in the soft tighten mode the nut follows the thread, so the TCP xy here is
    # the best estimate of where the stud really is
    record.update(drop_end_mm=round(start_z - z, 2), tilt_end_deg=round(tilt, 2),
                  force_end_n=force,
                  end_tcp_mm=rounded(frames.tcp_position(robot.flange_pose(), tcp_offset)))


def place_and_screw(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
                    poses: config.Poses, record: dict, approach_only: bool = False,
                    mark: Callable[[str], object] = no_mark) -> None:
    """Step 2: observation pose over the stud -> stud search, measured from close by
    (find_stud) -> to beside the approach pose,
    then the last bit always from the same side -> descend until contact -> press the nut up in
    the iris (seat_nut) -> spiral search for the thread (find_thread) -> screw (screw_on) ->
    release -> up to the close-up pose over the stud, one camera image of the result
    (snapshot "final").
    approach_only: stop above the thread tip and hold (centring check, no gripper).
    mark(event) at the steps (run_record.Timeline). The record holds every stage's TCP minus
    the computed target (*_minus_target_xy_mm), for the evaluation."""
    table_z = poses.table_z()
    stud = cfg["objects"]["stud"]
    place = cfg["place"]
    modes = cfg["robot"]["modes"]
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    observe = observation_pose(poses, "observe_stud", cfg)
    robot.set_mode(modes["normal"])
    vision.prepare(TARGET_STUD)
    # first look while creeping, as at the nut. Full run: this lifts the nut off the table,
    # with z leading less than usual (27.10.: nothing around the held nut needs a vertical exit)
    move_to(robot, observe, cfg, rough=True, lead_fraction=place["lift_lead_fraction"],
            max_speed_z_mm_s=place["lift_max_speed_z_mm_s"])
    mark("observe_stud")
    found = record["stud"] = find_stud(robot, vision, cfg, table_z, record, mark)
    mark("stud_located")
    target_xy = np.asarray(found["position_mm"][:2])

    def from_target(tcp) -> list:
        return rounded(np.asarray(tcp[:2], float) - target_xy)

    # The nut sits in the iris with its bottom face at the TCP plane
    tip_z = table_z + stud["top_height_mm"]
    approach = frames.flange_for_tcp([*found["position_mm"][:2],
                                      tip_z + place["approach_clearance_mm"]],
                                     config.rotation_down(cfg), tcp_offset)
    # First beside the target, then the last short move always in the same direction: the
    # play of the gripper's turning disk then always ends up on the same side (25.10.)
    if np.any(place["approach_from_mm"]):  # [0, 0]: straight to the approach pose
        beside = approach.copy()
        beside[:2, 3] += place["approach_from_mm"]
        move_to(robot, beside, cfg, lead=LEAD_XY)
    approach = move_to(robot, approach, cfg, precise=True)
    reached = frames.tcp_position(robot.flange_pose(), tcp_offset)
    record["approach_reached_tcp_mm"] = rounded(reached)
    record["approach_minus_target_xy_mm"] = from_target(reached)
    mark("approached")
    # Record only: how far the flange tilt moves a rigid TCP here (25.10.: 0-2 mm, varied from
    # run to run at the same place; a swinging-plate model built on it was dropped)
    record["approach_tilt_shift_mm"] = rounded(frames.tilt_shift_mm(
        robot.flange_pose(), config.rotation_down(cfg), tcp_offset))
    if approach_only:
        log.info("approach only: TCP %.1f mm above the thread tip; holding here",
                 reached[2] - tip_z)
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
    mark("contact")
    contact = robot.flange_pose()
    record["contact_tcp_mm"] = rounded(frames.tcp_position(contact, tcp_offset))
    record["contact_minus_target_xy_mm"] = from_target(record["contact_tcp_mm"])
    record["seat"] = {}
    # From here on the reference only goes down until the release (27.10.)
    contact, reference_z = seat_nut(robot, cfg, contact, free_fz, record["seat"])
    mark("seated")
    record["thread_search"] = {}
    contact, reference_z = find_thread(robot, cfg, contact, record["thread_search"],
                                       reference_z)
    mark("thread_found")
    # where the nut dropped onto the thread start: the stud as the iris felt it
    record["thread_minus_target_xy_mm"] = from_target(record["thread_search"]["lowest_tcp_mm"])
    record["screw"] = {}
    screw_on(robot, gripper, cfg, contact, free_fz, record["screw"], mark, reference_z)
    record["end_minus_target_xy_mm"] = from_target(record["screw"]["end_tcp_mm"])
    if place["open_while_settling"]:
        # 27.10. (Yannick): the iris opens during the release wait, ~1.5 s faster. The wait
        # was added on 23.10. because the soft arm dropped when the iris let go; since then
        # the tighten mode is 4x stiffer in z (600 N/m)
        mark("iris_opening")  # end of the cycle time (GUI timer)
        alongside(gripper.open, lambda: stiffen_where_it_is(robot, cfg))
    else:
        stiffen_where_it_is(robot, cfg)
        mark("iris_opening")
        gripper.open()
    mark("iris_opened")
    record["released_tcp_mm"] = rounded(frames.tcp_position(robot.flange_pose(), tcp_offset))
    # Up to the close-up pose over the stud, not back to the observation pose (27.10.: ~3 s
    # shorter, and the photo of the result is taken from 170 mm instead of ~400 mm)
    move_to(robot, close_up_pose(cfg, found["close_mm"][:2], table_z + stud["base_height_mm"]),
            cfg, lead=LEAD_Z)
    mark("back_up")
    vision.snapshot("final")  # nut on the stud (or not), for judging the run later
