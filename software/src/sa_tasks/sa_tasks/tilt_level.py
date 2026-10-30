"""Find the orientation correction that makes the gripper bottom plate level (28.10.).

  ros2 run sa_tasks tilt_level      # needs the robot (sa_robot) running; no gripper, no vision

The plate hangs lower on its -y_B side inside the tool, and the arm tilts the whole tool
outwards (away from the base), the other way round on each side of the table. With the stud
on the left (+y) both add up, on the right (-y) they cancel (THEORY 5, tilt about the TCP).
This tool measures the correction per side by eye: the TCP goes to tilt_level.start_gap_mm
above the table at the centre of each area (where the camera axis of the observation pose
meets the table, or a TCP point [x, y] given in tilt_level.points), then the plate is turned
about the TCP in steps until it looks level (feeler gauge or paper strip under both edges, or
light through the gap).

THE TABLE MUST BE CLEAR (no stud, no nut) under the plate. Commands, each after Enter:
  Enter          show the commanded correction, the measured tilt and the TCP height
  + / -          -y edge up / down by one step (turned about the TCP)
  0              back to no correction
  step <deg>     step size, e.g. step 0.25
  down / up      TCP z_step_mm lower / higher (refused below min_gap_mm, measured)
  mode <name>    stiffness preset: normal, screw, tighten (screw/tighten: soft z, the arm sags)
  save [note]    append the current state to <log_dir>/tilt_level.yaml
  next           up to safe_height_mm, level, on to the next area
  q              up to safe_height_mm, level, end
Correction > 0: the plate's -y_B edge goes up (= R_x(-angle), the TCP swings towards -y_B).
"""
import logging
import threading
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, frames, sequences
from sa_tasks.ros_interfaces import FrankaRobot
from sa_tasks.trajectory import LEAD_XY, LEAD_Z

log = logging.getLogger("sa_tasks")


def area_centre(robot: FrankaRobot, cfg: dict, poses: config.Poses, name: str,
                table_z: float) -> np.ndarray:
    """Where the camera axis of observation pose `name` meets the table (base frame, xy)."""
    sequences.move_to(robot, sequences.observation_pose(poses, name, cfg), cfg)
    camera, distance = sequences.camera_and_distance(robot, cfg, table_z)
    return (camera[:3, 3] + distance * camera[:3, 2])[:2]


def state_of(robot: FrankaRobot, cfg: dict, table_z: float, correction_deg: float) -> dict:
    """Commanded correction vs. the measured tilt, both as '-y edge up' angles."""
    flange = robot.flange_pose()
    about_x, about_y = frames.axis_tilt_deg(flange[:3, :3])
    measured = -about_x  # -y edge up, measured (commanded R_down has 0)
    tcp = frames.tcp_position(flange, cfg["frames"]["flange_to_tcp_mm"])
    return {"commanded_minus_y_up_deg": round(correction_deg, 2),
            "measured_minus_y_up_deg": round(measured, 2),
            # what the arm adds on its own (spring deflection of the controller)
            "arm_minus_y_up_deg": round(measured - correction_deg, 2),
            "measured_tilt_about_y_deg": round(about_y, 2),
            "tilt_deg": round(frames.tilt_deg(flange[:3, :3]), 2),
            "tcp_mm": sequences.rounded(tcp),
            "tcp_above_table_mm": round(float(tcp[2] - table_z), 1)}


def show(state: dict) -> None:
    print(f"  commanded {state['commanded_minus_y_up_deg']:+.2f} deg | measured "
          f"{state['measured_minus_y_up_deg']:+.2f} deg (arm "
          f"{state['arm_minus_y_up_deg']:+.2f}) | about y {state['measured_tilt_about_y_deg']:+.2f}"
          f" deg | TCP {state['tcp_above_table_mm']:.1f} mm above the table")


def main():
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    cfg = config.load_tasks()
    t = cfg["tilt_level"]
    poses = config.Poses()
    table_z = poses.table_z()
    r_down = config.rotation_down(cfg)
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    log_path = config.PACKAGE_DIR.parents[2] / cfg["log_dir"] / "tilt_level.yaml"
    rclpy.init()
    node = rclpy.create_node("sa_tasks_tilt_level")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    print(__doc__.split("\n\n", 2)[2])
    mode_name = "normal"
    try:
        robot = FrankaRobot(node, cfg)
        robot.set_mode(cfg["robot"]["modes"][mode_name])
        for point in t["points"]:
            name = point if isinstance(point, str) else "x {:.0f} y {:.0f}".format(*point)
            input(f"Enter: to the centre of the {name} area (TABLE CLEAR?) ")
            # a pose name: where its camera axis meets the table; [x, y]: that TCP point
            centre = (area_centre(robot, cfg, poses, point, table_z) if isinstance(point, str)
                      else np.asarray(point, float))
            side = ("+y (left)" if centre[1] > 0 else "-y (right)" if centre[1] < 0
                    else "y = 0 (centre)")
            gap, correction, step = t["start_gap_mm"], 0.0, t["step_deg"]

            def goal() -> np.ndarray:
                return frames.flange_for_tcp([centre[0], centre[1], table_z + gap],
                                             frames.raise_minus_y_edge(r_down, correction),
                                             tcp_offset)
            sequences.move_to(robot, frames.flange_for_tcp(
                [centre[0], centre[1], table_z + t["safe_height_mm"]], r_down, tcp_offset), cfg)
            sequences.move_to(robot, goal(), cfg, lead=LEAD_XY)
            print(f"{name} area, stud side {side}: x {centre[0]:.1f} y {centre[1]:.1f} mm")
            show(state_of(robot, cfg, table_z, correction))
            while True:
                command = input(f"[{name}, step {step:g} deg] > ").strip().split()
                word = command[0] if command else ""
                if word in ("next", "q"):
                    break
                moved = True
                if word in ("+", "-"):
                    new = correction + (step if word == "+" else -step)
                    if abs(new) > t["max_correction_deg"]:
                        print(f"  refused: more than {t['max_correction_deg']} deg")
                        moved = False
                    else:
                        correction = new
                elif word == "0":
                    correction = 0.0
                elif word == "step" and len(command) == 2:
                    step, moved = float(command[1]), False
                elif word in ("down", "up"):
                    now = state_of(robot, cfg, table_z, correction)["tcp_above_table_mm"]
                    if word == "down" and now - t["z_step_mm"] < t["min_gap_mm"]:
                        print(f"  refused: the TCP would be below {t['min_gap_mm']} mm")
                        moved = False
                    else:
                        gap += t["z_step_mm"] if word == "up" else -t["z_step_mm"]
                elif word == "mode" and len(command) == 2 \
                        and command[1] in cfg["robot"]["modes"]:
                    mode_name = command[1]
                    robot.set_mode(cfg["robot"]["modes"][mode_name])
                    moved = False
                elif word == "save":
                    entry = {"time": datetime.now().isoformat(timespec="seconds"),
                             "area": name, "side": side, "centre_xy_mm": sequences.rounded(centre),
                             "mode": mode_name, "note": " ".join(command[1:]),
                             **state_of(robot, cfg, table_z, correction)}
                    history = (yaml.safe_load(log_path.read_text())
                               if log_path.exists() else None) or []
                    history.append(entry)
                    log_path.write_text(yaml.safe_dump(history, sort_keys=False))
                    print(f"  saved to {log_path}")
                    moved = False
                elif word:
                    print("  unknown command")
                    moved = False
                else:
                    moved = False
                if moved:
                    sequences.move_to(robot, goal(), cfg)
                show(state_of(robot, cfg, table_z, correction))
            correction, mode_name = 0.0, "normal"
            robot.set_mode(cfg["robot"]["modes"][mode_name])  # stiff again before moving up
            sequences.move_to(robot, frames.flange_for_tcp(
                [centre[0], centre[1], table_z + t["safe_height_mm"]], r_down, tcp_offset),
                cfg, lead=LEAD_Z)
            if word == "q":
                break
        input("Enter: end (the arm stays where it is) ")
    except KeyboardInterrupt:
        log.info("aborted")
    finally:
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
