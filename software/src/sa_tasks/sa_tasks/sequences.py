"""Step 1 (pick the nut) and step 2 (place it on the stud and screw it on). No ROS.

docs/PLAN.md Section 4, docs/THEORY.md Sections 3 and 5. The sequences only use the
Robot, Gripper and Vision interfaces (interfaces.py), so they run against mocks.
"""
import logging
from typing import Optional

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


class ToolRobot:
    """The robot seen in the gripper frame G instead of the flange frame F.

    G = F tilted by the measured mount tilt (same origin), so commanding G level keeps the
    bottom plate (and the camera on it) level. Identity transform = plain flange frame.
    """

    def __init__(self, robot: Robot, t_flange_tool: np.ndarray):
        self.robot = robot
        self.t_flange_tool = t_flange_tool
        self.t_tool_flange = frames.invert(t_flange_tool)

    def flange_pose(self) -> np.ndarray:  # pose of G, named like the Robot interface
        return self.robot.flange_pose() @ self.t_flange_tool

    def force_z(self) -> float:
        return self.robot.force_z()

    def set_mode(self, mode: int) -> None:
        self.robot.set_mode(mode)

    def follow(self, path, stop_on_contact_n=None) -> bool:
        return self.robot.follow([Waypoint(w.time_s, w.t_base_flange @ self.t_tool_flange)
                                  for w in path], stop_on_contact_n)

    def wait_until_arrived(self, goal: np.ndarray, tolerance_mm: float) -> None:
        self.robot.wait_until_arrived(goal @ self.t_tool_flange, tolerance_mm)


def move_to(robot: Robot, goal: np.ndarray, cfg: dict, lead: Optional[str] = None,
            extra_tolerance_mm: float = 0.0) -> None:
    """One continuous move of the flange from where it is to goal, then wait for arrival."""
    path = plan_move(robot.flange_pose(), goal, config.motion_limits(cfg),
                     cfg["robot"]["stream_rate_hz"], lead)
    robot.follow(path)
    robot.wait_until_arrived(goal, cfg["robot"]["arrival_tolerance_mm"] + extra_tolerance_mm)


def orientation_error(robot: Robot, rotation: np.ndarray) -> dict:
    """Measured tilt (tool axis vs. vertical) and angle to the commanded orientation, deg."""
    r = robot.flange_pose()[:3, :3]
    return {"tilt_deg": round(frames.tilt_deg(r), 3),
            "to_commanded_deg": round(frames.rotation_angle_deg(r, rotation), 3)}


def observation_pose(poses: config.Poses, name: str, cfg: dict) -> np.ndarray:
    """Taught position with the fixed orientation (quaternion (1, 0, 0, 0)); only the
    taught position counts, a grossly tilted teach is refused.

    With robot.observation_height_from set, z is taken from that pose, so both
    observation poses share one height (same image scale for nut and stud).
    """
    taught = poses.get(name)
    tilt = frames.tilt_deg(taught[:3, :3])
    if tilt > cfg["robot"]["max_taught_tilt_deg"]:
        raise SequenceError(f"taught pose '{name}' is tilted by {tilt:.1f} deg, teach it again")
    position = taught[:3, 3].copy()
    height_from = cfg["robot"]["observation_height_from"]
    if height_from:
        position[2] = poses.get(height_from)[2, 3]
    return frames.pose(config.rotation_down(cfg), position)


def locate(robot: Robot, vision: Vision, cfg: dict, table_z: float, target: str,
           size_plane_mm: float, centre_plane_mm: float) -> dict:
    """Detect the object from the current pose and return its position in the base frame.

    size_plane_mm: height of the face whose size the detection checks (sets d for the
    size filter); centre_plane_mm: height of the detected centre (ray-plane intersection).
    """
    t_base_camera = robot.flange_pose() @ config.camera_in_flange(cfg)
    distance = frames.distance_along_axis(t_base_camera, table_z + size_plane_mm)
    observation = vision.observe(target, distance)
    plane_z = table_z + centre_plane_mm
    point = frames.object_in_base(observation.u_undistorted, observation.v_undistorted,
                                  observation.k, t_base_camera, plane_z)
    axis = frames.object_in_base(observation.k[0, 2], observation.k[1, 2], observation.k,
                                 t_base_camera, plane_z)
    offset = float(np.linalg.norm(point[:2] - axis[:2]))
    if offset > cfg["vision"]["max_offset_from_axis_mm"]:
        raise SequenceError(f"{target} found {offset:.0f} mm from the camera axis, outside "
                            f"the work area (max {cfg['vision']['max_offset_from_axis_mm']} mm)")
    point[:2] += np.asarray(cfg["frames"]["target_correction_mm"], float)
    log.info("%s at x %.1f y %.1f z %.1f mm (base), %.0f mm from the camera axis, d %.0f mm",
             target, *point, offset, distance)
    return {"target": target, "position_mm": point.tolist(), "offset_from_axis_mm": offset,
            "camera_axis_mm": axis.tolist(), "plane_distance_mm": distance,
            "pixel_undistorted": [observation.u_undistorted, observation.v_undistorted]}


def candidate_positions(robot: Robot, vision: Vision, cfg: dict, table_z: float, target: str,
                        size_plane_mm: float, centre_plane_mm: float, lock: bool) -> tuple:
    """All objects of one view at their base positions, in the rank order of vision_node
    (lock: see Vision.view). Returns (camera axis point on the centre plane, d, [dict per
    object])."""
    t_base_camera = robot.flange_pose() @ config.camera_in_flange(cfg)
    distance = frames.distance_along_axis(t_base_camera, table_z + size_plane_mm)
    view = vision.view(target, distance, lock)
    plane_z = table_z + centre_plane_mm
    axis = frames.object_in_base(view.k[0, 2], view.k[1, 2], view.k, t_base_camera, plane_z)
    objects = []
    for rank, candidate in enumerate(view.candidates, 1):
        point = frames.object_in_base(candidate.u_undistorted, candidate.v_undistorted, view.k,
                                      t_base_camera, plane_z)
        offset = float(np.linalg.norm(point[:2] - axis[:2]))
        objects.append({"rank": rank, "position_mm": [round(float(v), 2) for v in point],
                        "offset_from_axis_mm": round(offset, 2),
                        "size_mm": round(candidate.size_mm, 2),
                        "width_ratio": round(candidate.width_ratio, 3),
                        "shape_ok": bool(candidate.shape_ok)})
    return axis, distance, objects


def find_stud(robot: Robot, vision: Vision, cfg: dict, table_z: float, observe: np.ndarray,
              record: dict) -> dict:
    """Stud search (THEORY 2.0): the stud is a clean square only when seen from directly
    above. All objects seen from the observation pose are tried in rank order: the camera
    axis is moved over the object (from the measured pose, so the steady-state error of the
    controller is corrected too) until it is within centred_tolerance_mm, then its shape is
    judged there. After a move the largest object within lost_radius_mm of the axis counts
    (dark spots on the stud itself can be separate small blobs nearer to the axis). The
    first object that passes is the stud; its position comes from that centred image,
    where the parallax of the tall stud is negligible."""
    stud = cfg["objects"]["stud"]
    search = cfg["place"]["stud_search"]
    planes = (stud["base_height_mm"], stud["centre_height_mm"])
    # Choosing among several objects: locked on objects confirmed over several images
    axis, _, seen = candidate_positions(robot, vision, cfg, table_z, TARGET_STUD, *planes,
                                        lock=True)
    record["candidates"] = seen
    limit = cfg["vision"]["max_offset_from_axis_mm"]
    candidates = [c for c in seen if c["offset_from_axis_mm"] <= limit]
    if not candidates:
        raise SequenceError(f"no object within {limit} mm of the camera axis at the stud "
                            f"observation pose ({len(seen)} in view)")
    candidates = candidates[:search["max_candidates"]]
    log.info("Step 2: %d object(s) in view, trying %d in rank order", len(seen), len(candidates))
    attempts = record["attempts"] = []
    goal = observe.copy()  # last commanded pose = where `axis` was measured
    for candidate in candidates:
        position = np.asarray(candidate["position_mm"][:2], float)
        for move in range(1, search["max_centring_moves"] + 1):
            goal[:2, 3] += position - axis[:2]
            move_to(robot, goal, cfg)
            # Centred: the median over the last images, no lock (worked well, 20.10.)
            axis, distance, now = candidate_positions(robot, vision, cfg, table_z, TARGET_STUD,
                                                      *planes, lock=False)
            near = [c for c in now if c["offset_from_axis_mm"] <= search["lost_radius_mm"]]
            nearest = max(near, key=lambda c: c["size_mm"]) if near else None
            attempts.append({"candidate": candidate["rank"], "move": move, "nearest": nearest})
            if nearest is None:
                log.warning("candidate #%d lost after centring move %d", candidate["rank"], move)
                break
            position = np.asarray(nearest["position_mm"][:2], float)
            if nearest["offset_from_axis_mm"] > search["centred_tolerance_mm"]:
                continue
            if not nearest["shape_ok"]:
                log.info("candidate #%d seen from above: width ratio %.2f, %.1f mm: not a stud",
                         candidate["rank"], nearest["width_ratio"], nearest["size_mm"])
                break
            point = np.asarray(nearest["position_mm"], float)
            point[:2] += np.asarray(cfg["frames"]["target_correction_mm"], float)
            log.info("stud = candidate #%d (width ratio %.2f, %.1f mm) at x %.1f y %.1f mm, "
                     "%.1f mm from the camera axis after %d move(s)", candidate["rank"],
                     nearest["width_ratio"], nearest["size_mm"], point[0], point[1],
                     nearest["offset_from_axis_mm"], move)
            return {"target": TARGET_STUD, "position_mm": point.tolist(),
                    "uncorrected_mm": nearest["position_mm"],
                    "observed_mm": candidate["position_mm"],
                    "offset_from_axis_mm": nearest["offset_from_axis_mm"],
                    "camera_axis_mm": axis.tolist(), "plane_distance_mm": distance,
                    "candidate": candidate["rank"], "centring_moves": move}
        else:
            log.warning("candidate #%d not centred within %d moves", candidate["rank"],
                        search["max_centring_moves"])
    raise SequenceError(f"none of the {len(candidates)} object(s) tried is a stud")


def tcp_move(robot: Robot, cfg: dict, found: dict) -> dict:
    """Split the horizontal move of the TCP from the observation pose to the detected object
    into the part the image gives (object relative to the camera axis, incl. the target
    correction) and the fixed part of the mount (TCP -> camera axis on the object plane,
    about +44.4 mm in x_B with the camera in front; THEORY 3.3). Logged and recorded only,
    the target itself comes from the frame chain."""
    tcp_now = frames.tcp_position(robot.flange_pose(), cfg["frames"]["flange_to_tcp_mm"])[:2]
    axis = np.asarray(found["camera_axis_mm"][:2], float)
    from_image = np.asarray(found["position_mm"][:2], float) - axis
    mount = axis - tcp_now
    total = from_image + mount
    log.info("TCP move x %.1f y %.1f mm = %s from the camera axis (image) x %.1f y %.1f "
             "+ TCP -> camera axis (mount) x %.1f y %.1f", *total, found["target"],
             *from_image, *mount)
    return {"total_xy_mm": [round(float(v), 2) for v in total],
            "from_image_xy_mm": [round(float(v), 2) for v in from_image],
            "tcp_to_camera_axis_xy_mm": [round(float(v), 2) for v in mount]}


def pick(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
         poses: config.Poses, locate_only: bool = False, record: Optional[dict] = None) -> dict:
    """Step 1: observation pose -> detect the nut -> one move down until the gripper rests
    flat on the table with the iris centred over the nut -> close -> back up.

    locate_only: stop after the detection (verification of the target computation:
    compare the printed nut position with the hand-measured one).
    record: filled step by step, so an abort (Ctrl-C) keeps what was measured so far."""
    record = {} if record is None else record
    record["step"] = "pick"
    robot = ToolRobot(robot, config.tool_in_flange(cfg, poses))
    table_z = poses.table_z()
    nut = cfg["objects"]["nut"]
    rotation = config.rotation_down(cfg)
    observe = observation_pose(poses, "observe_nut", cfg)

    robot.set_mode(cfg["robot"]["modes"]["normal"])
    vision.prepare(TARGET_NUT)
    gripper.open()
    log.info("Step 1: moving to the nut observation pose")
    move_to(robot, observe, cfg)
    record["orientation"] = {"observe": orientation_error(robot, rotation)}
    found = locate(robot, vision, cfg, table_z, TARGET_NUT, nut["height_mm"],
                   nut["centre_height_mm"])
    record["nut"] = found
    record["tcp_move"] = tcp_move(robot, cfg, found)
    if locate_only:
        log.info("Step 1 (locate only): nut at x %.1f y %.1f mm, no descent",
                 *found["position_mm"][:2])
        record["locate_only"] = True
        return record

    press = cfg["pick"]["press_depth_mm"]
    tcp = [found["position_mm"][0], found["position_mm"][1], table_z - press]
    record["tcp_target_mm"] = tcp
    spiral = cfg["pick"]["spiral"]
    # Down to just above the nut, then without a stop a spiral search down onto the table:
    # if the target is off by up to the spiral radius, the plate slides over the nut until
    # the open iris passes over it and the nut drops in (THEORY 5)
    start = frames.flange_for_tcp([tcp[0], tcp[1], table_z + spiral["start_height_mm"]],
                                  rotation, cfg["frames"]["flange_to_tcp_mm"])
    log.info("Step 1: down to the nut (TCP %.1f, %.1f), spiral from table + %.0f mm "
             "(radius %.1f mm, %.1f turns) to table - %.1f mm", tcp[0], tcp[1],
             spiral["start_height_mm"], spiral["radius_mm"], spiral["turns"], press)
    rate = cfg["robot"]["stream_rate_hz"]
    approach = plan_move(robot.flange_pose(), start, config.motion_limits(cfg), rate, LEAD_XY)
    search = spiral_down(start, spiral["start_height_mm"] + press, spiral["radius_mm"],
                         spiral["turns"], spiral["duration_s"], rate)
    robot.follow(chain(approach, search))
    # If the iris misses, the bottom plate stands on the nut instead of the table: accept
    # that (the arm stands still) and record it instead of aborting
    robot.wait_until_arrived(search[-1].t_base_flange, cfg["robot"]["arrival_tolerance_mm"]
                             + press + nut["height_mm"])
    record["spiral_end_tcp_mm"] = [round(float(v), 2) for v in frames.tcp_position(
        search[-1].t_base_flange, cfg["frames"]["flange_to_tcp_mm"])]
    record["orientation"]["landed"] = orientation_error(robot, rotation)
    landed = frames.tcp_position(robot.flange_pose(), cfg["frames"]["flange_to_tcp_mm"])
    above_table = float(landed[2] - table_z)
    log.info("landed: TCP x %.1f y %.1f mm, %.1f mm above the table (target x %.1f y %.1f)",
             landed[0], landed[1], above_table, tcp[0], tcp[1])
    record["landed_tcp_mm"] = [round(float(v), 2) for v in landed]
    record["landed_above_table_mm"] = round(above_table, 2)
    if above_table > nut["height_mm"] / 2:
        log.warning("gripper stands %.1f mm above the table: iris missed the nut?", above_table)
    if cfg["pick"]["close_iris"]:
        gripper.close()
    log.info("Step 1: back to the observation pose")
    move_to(robot, observe, cfg, lead=LEAD_Z)
    return record


def position_check(found: dict, reached_tcp: np.ndarray) -> dict:
    """Stud xy at three stages (approach_only): 1 seen from the observation pose (off-axis,
    locked), 2 seen with the camera centred over it (the target, before target_correction_mm),
    3 the TCP measured at the approach pose. 2 - 1 large: the camera model is off (same
    stud, two views); 3 - 2: what the robot did not reach. TCP visibly off the real stud
    although 3 - 2 is small: constant camera -> TCP offset (target_correction_mm)."""
    observed = np.asarray(found["observed_mm"][:2], float)
    centred = np.asarray(found["uncorrected_mm"][:2], float)
    tcp = np.asarray(reached_tcp[:2], float)
    rows = [("1 observation pose (off-axis)", observed), ("2 centred (target)", centred),
            ("3 TCP reached", tcp)]
    for label, xy in rows:
        log.info("check %-30s x %7.2f  y %7.2f mm", label, *xy)
    log.info("check 2 - 1 (camera model)       x %+6.2f  y %+6.2f mm", *(centred - observed))
    log.info("check 3 - 2 (robot)              x %+6.2f  y %+6.2f mm", *(tcp - centred))
    log.info("check: now measure the bottom plate centre against the stud axis by hand")
    check = {label.split()[0]: [round(float(v), 2) for v in xy] for label, xy in rows}
    check["centred_minus_observed_mm"] = [round(float(v), 2) for v in centred - observed]
    check["tcp_minus_centred_mm"] = [round(float(v), 2) for v in tcp - centred]
    return check


def place_and_screw(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
                    poses: config.Poses, locate_only: bool = False,
                    approach_only: bool = False, record: Optional[dict] = None) -> dict:
    """Step 2: observation pose over the stud -> detect it -> one move to just above the
    thread -> descend until contact -> screw (compliant) -> release -> back up.

    locate_only: stop after the detection (test of the stud detection and its position).
    approach_only: stop at the approach pose above the thread tip and hold it there (check
    the centring by eye; no contact search, screwing or gripper command).
    record: filled step by step, so an abort (Ctrl-C) keeps what was measured so far."""
    record = {} if record is None else record
    record["step"] = "place_and_screw"
    robot = ToolRobot(robot, config.tool_in_flange(cfg, poses))
    table_z = poses.table_z()
    stud = cfg["objects"]["stud"]
    place = cfg["place"]
    modes = cfg["robot"]["modes"]
    rate = cfg["robot"]["stream_rate_hz"]
    rotation = config.rotation_down(cfg)
    observe = observation_pose(poses, "observe_stud", cfg)

    robot.set_mode(modes["normal"])
    vision.prepare(TARGET_STUD)
    log.info("Step 2: moving to the stud observation pose")
    move_to(robot, observe, cfg)
    found = find_stud(robot, vision, cfg, table_z, observe, record)
    record["stud"] = found
    record["tcp_move"] = tcp_move(robot, cfg, found)
    if locate_only:
        log.info("Step 2 (locate only): stud at x %.1f y %.1f mm, no descent",
                 *found["position_mm"][:2])
        record["locate_only"] = True
        return record

    # The nut sits in the iris with its bottom face at the TCP plane
    tcp = [found["position_mm"][0], found["position_mm"][1],
           table_z + stud["top_height_mm"] + place["approach_clearance_mm"]]
    record["approach_tcp_mm"] = tcp
    approach = frames.flange_for_tcp(tcp, rotation, cfg["frames"]["flange_to_tcp_mm"])
    log.info("Step 2: one move to %.1f mm above the thread", place["approach_clearance_mm"])
    move_to(robot, approach, cfg, lead=LEAD_XY)
    if approach_only:
        reached = frames.tcp_position(robot.flange_pose(), cfg["frames"]["flange_to_tcp_mm"])
        error = reached[:2] - np.asarray(tcp[:2])
        log.info("Step 2 (approach only): TCP at x %.1f y %.1f z %.1f mm (%.1f mm above the "
                 "thread tip), %+.1f / %+.1f mm from the stud; holding here", *reached,
                 reached[2] - table_z - stud["top_height_mm"], *error)
        record["approach_only"] = True
        record["approach_reached_tcp_mm"] = [round(float(v), 2) for v in reached]
        record["check"] = position_check(found, reached)
        return record

    descent = straight_down(approach, place["max_descend_mm"], place["descend_speed_mm_s"], rate)
    if not robot.follow(descent, stop_on_contact_n=place["contact_force_n"]):
        move_to(robot, approach, cfg, lead=LEAD_Z)
        raise SequenceError(f"no contact within {place['max_descend_mm']} mm below the "
                            "approach pose: nut not on the thread?")
    contact = robot.flange_pose()
    record["contact_tcp_mm"] = frames.tcp_position(
        contact, cfg["frames"]["flange_to_tcp_mm"]).tolist()
    log.info("Step 2: contact at TCP z = table + %.1f mm, screwing",
             frames.tcp_position(contact, cfg["frames"]["flange_to_tcp_mm"])[2] - table_z)

    screw = place["screw"]
    robot.set_mode(modes["float"])
    below = contact.copy()
    below[2, 3] -= screw["follow_depth_mm"]
    robot.follow([Waypoint(0.0, below)])
    gripper.rotate(screw["direction"], screw["duration_ms"], screw["speed"])
    gripper.open()
    # Reference back to where the arm is before the stiff mode returns, else it would
    # jump towards the reference below the thread
    robot.follow([Waypoint(0.0, robot.flange_pose())])
    robot.set_mode(modes["normal"])
    log.info("Step 2: back to the observation pose")
    move_to(robot, observe, cfg, lead=LEAD_Z)
    return record
