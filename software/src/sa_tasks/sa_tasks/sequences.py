"""Step 1 (pick the nut) and step 2 (place it on the stud and screw it on). No ROS.

docs/PLAN.md Section 4, docs/THEORY.md Sections 3 and 5. The sequences only use the
Robot, Gripper and Vision interfaces (interfaces.py), so they run against mocks.
"""
import logging
from typing import Optional

import numpy as np

from sa_tasks import config, frames
from sa_tasks.interfaces import Gripper, Robot, Vision
from sa_tasks.trajectory import LEAD_XY, LEAD_Z, Waypoint, plan_move, straight_down

log = logging.getLogger("sa_tasks")

TARGET_NUT = "nut"
TARGET_STUD = "stud"


class SequenceError(RuntimeError):
    """A step failed; the robot stays where it is (no automatic recovery)."""


def move_to(robot: Robot, goal: np.ndarray, cfg: dict, lead: Optional[str] = None,
            extra_tolerance_mm: float = 0.0) -> None:
    """One continuous move of the flange from where it is to goal, then wait for arrival."""
    path = plan_move(robot.flange_pose(), goal, config.motion_limits(cfg),
                     cfg["robot"]["stream_rate_hz"], lead)
    robot.follow(path)
    robot.wait_until_arrived(goal, cfg["robot"]["arrival_tolerance_mm"] + extra_tolerance_mm)


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
            "plane_distance_mm": distance,
            "pixel_undistorted": [observation.u_undistorted, observation.v_undistorted]}


def pick(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
         poses: config.Poses) -> dict:
    """Step 1: observation pose -> detect the nut -> one move down until the gripper rests
    flat on the table with the iris centred over the nut -> close -> back up."""
    table_z = poses.table_z()
    nut = cfg["objects"]["nut"]
    rotation = config.rotation_down(cfg)
    observe = observation_pose(poses, "observe_nut", cfg)

    robot.set_mode(cfg["robot"]["modes"]["normal"])
    gripper.open()
    log.info("Step 1: moving to the nut observation pose")
    move_to(robot, observe, cfg)
    found = locate(robot, vision, cfg, table_z, TARGET_NUT, nut["height_mm"],
                   nut["centre_height_mm"])

    press = cfg["pick"]["press_depth_mm"]
    tcp = [found["position_mm"][0], found["position_mm"][1], table_z - press]
    grasp = frames.flange_for_tcp(tcp, rotation, cfg["frames"]["flange_to_tcp_mm"])
    log.info("Step 1: one move down onto the nut (TCP %.1f, %.1f, table - %.1f mm)",
             tcp[0], tcp[1], press)
    move_to(robot, grasp, cfg, lead=LEAD_XY, extra_tolerance_mm=press)
    if cfg["pick"]["close_iris"]:
        gripper.close()
    log.info("Step 1: back to the observation pose")
    move_to(robot, observe, cfg, lead=LEAD_Z)
    return {"step": "pick", "nut": found, "tcp_target_mm": tcp}


def place_and_screw(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
                    poses: config.Poses) -> dict:
    """Step 2: observation pose over the stud -> detect it -> one move to just above the
    thread -> descend until contact -> screw (compliant) -> release -> back up."""
    table_z = poses.table_z()
    stud = cfg["objects"]["stud"]
    place = cfg["place"]
    modes = cfg["robot"]["modes"]
    rate = cfg["robot"]["stream_rate_hz"]
    rotation = config.rotation_down(cfg)
    observe = observation_pose(poses, "observe_stud", cfg)

    robot.set_mode(modes["normal"])
    log.info("Step 2: moving to the stud observation pose")
    move_to(robot, observe, cfg)
    found = locate(robot, vision, cfg, table_z, TARGET_STUD, stud["base_height_mm"],
                   stud["base_height_mm"])

    # The nut sits in the iris with its bottom face at the TCP plane
    tcp = [found["position_mm"][0], found["position_mm"][1],
           table_z + stud["top_height_mm"] + place["approach_clearance_mm"]]
    approach = frames.flange_for_tcp(tcp, rotation, cfg["frames"]["flange_to_tcp_mm"])
    log.info("Step 2: one move to %.1f mm above the thread", place["approach_clearance_mm"])
    move_to(robot, approach, cfg, lead=LEAD_XY)

    descent = straight_down(approach, place["max_descend_mm"], place["descend_speed_mm_s"], rate)
    if not robot.follow(descent, stop_on_contact_n=place["contact_force_n"]):
        move_to(robot, approach, cfg, lead=LEAD_Z)
        raise SequenceError(f"no contact within {place['max_descend_mm']} mm below the "
                            "approach pose: nut not on the thread?")
    contact = robot.flange_pose()
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
    return {"step": "place_and_screw", "stud": found, "approach_tcp_mm": tcp,
            "contact_tcp_mm": frames.tcp_position(contact,
                                                  cfg["frames"]["flange_to_tcp_mm"]).tolist()}
