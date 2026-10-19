"""Step 1 (pick the nut) and step 2 (place it on the stud and screw it on). No ROS.

docs/PLAN.md Section 4, docs/THEORY.md Sections 3 and 5. The sequences only use the
Robot, Gripper and Vision interfaces (interfaces.py), so they run against mocks.
"""
import logging
import time
from typing import Optional

import numpy as np

from sa_tasks import config, frames
from sa_tasks.interfaces import Gripper, Robot, Vision
from sa_tasks.trajectory import LEAD_XY, LEAD_Z, Waypoint, plan_move, straight_down

log = logging.getLogger("sa_tasks")

TARGET_NUT = "nut"
ALIGN_POLL_S = 0.05
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


def wait_aligned(robot: Robot, rotation: np.ndarray, tolerance_deg: float,
                 timeout_s: float) -> dict:
    """Wait until the measured orientation is within tolerance of the commanded one (or the
    timeout ends: the impedance controller may keep a small steady-state error)."""
    deadline = time.monotonic() + timeout_s
    error = orientation_error(robot, rotation)
    while error["to_commanded_deg"] > tolerance_deg and time.monotonic() < deadline:
        time.sleep(ALIGN_POLL_S)
        error = orientation_error(robot, rotation)
    log.info("aligned: tilt %.2f deg, %.2f deg from the commanded orientation%s",
             error["tilt_deg"], error["to_commanded_deg"],
             "" if error["to_commanded_deg"] <= tolerance_deg else " (timeout)")
    return error


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
    grasp = frames.flange_for_tcp(tcp, rotation, cfg["frames"]["flange_to_tcp_mm"])
    align = cfg["pick"]["align"]
    if align["height_mm"] > 0:
        # Intermediate stop straight above the nut: let the orientation settle, then go
        # vertically down (test whether a tilt on the way down causes the misses)
        above = frames.flange_for_tcp([tcp[0], tcp[1], table_z + align["height_mm"]], rotation,
                                      cfg["frames"]["flange_to_tcp_mm"])
        log.info("Step 1: move above the nut (TCP %.1f, %.1f, table + %.0f mm)",
                 tcp[0], tcp[1], align["height_mm"])
        move_to(robot, above, cfg, lead=LEAD_XY)
        record["orientation"]["aligned"] = wait_aligned(robot, rotation, align["tolerance_deg"],
                                                        align["timeout_s"])
        log.info("Step 1: straight down onto the table (table - %.1f mm)", press)
    else:
        log.info("Step 1: one move down onto the nut (TCP %.1f, %.1f, table - %.1f mm)",
                 tcp[0], tcp[1], press)
    # If the iris misses, the bottom plate stands on the nut instead of the table: accept
    # that (the arm stands still) and record it instead of aborting
    move_to(robot, grasp, cfg, lead=LEAD_XY, extra_tolerance_mm=press + nut["height_mm"])
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


def place_and_screw(robot: Robot, gripper: Gripper, vision: Vision, cfg: dict,
                    poses: config.Poses) -> dict:
    """Step 2: observation pose over the stud -> detect it -> one move to just above the
    thread -> descend until contact -> screw (compliant) -> release -> back up."""
    robot = ToolRobot(robot, config.tool_in_flange(cfg, poses))
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
    move = tcp_move(robot, cfg, found)

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
    return {"step": "place_and_screw", "stud": found, "tcp_move": move, "approach_tcp_mm": tcp,
            "contact_tcp_mm": frames.tcp_position(contact,
                                                  cfg["frames"]["flange_to_tcp_mm"]).tolist()}
