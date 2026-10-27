"""Measure where the iris really is relative to what the camera computes, at the stud (26.10.).

hand_eye fits only the camera rotation (with one fixed orientation the camera position cannot
be told from the board position, so it stays at the CAD value). Errors of the camera position,
of the projection centre and of the iris centre against the flange axis do not depend on the
distance, and with the one fixed orientation they add up to one constant xy vector in the
base frame. This tool measures that vector with the real stud and the real iris:

  1. sa_place_approach (or place + screw): the stud is measured from close by and written to
     the run record (place.stud.close_mm); stop it once it holds above the thread
  2. Desk: programming mode, guide the gripper down by hand until the bare thread stands
     centred in the open iris (no nut), let go (the brakes hold it)
  3. Desk: execution mode, then  ros2 launch software/launch/robot_state.launch.py
  4. ros2 run sa_tasks tcp_offset

It reads the stud of the newest run record and the TCP now and prints TCP - stud = the
correction the target needs (frames.target_correction), with the current camera model.
Repeat at a few table positions: the same vector everywhere means one constant correction
fits. Each result is appended to <log_dir>/tcp_offset.yaml.
"""
import threading
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, frames, sequences
from sa_tasks.ros_interfaces import RobotState


def newest_stud(directory) -> tuple:
    """(record file name, place.stud) of the newest run record with a close-up stud."""
    for path in sorted(directory.glob("*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            stud = (yaml.safe_load(path.read_text()).get("place") or {}).get("stud") or {}
        except (yaml.YAMLError, AttributeError):
            continue
        if "close_mm" in stud:
            return path.name, stud
    raise RuntimeError(f"no run record with place.stud.close_mm in {directory}")


def main():
    cfg = config.load_tasks()
    directory = config.PACKAGE_DIR.parents[2] / cfg["log_dir"]
    record_name, stud = newest_stud(directory)
    table_z = config.Poses().table_z()
    tip_z = table_z + cfg["objects"]["stud"]["top_height_mm"]
    min_depth = cfg["tcp_offset"]["min_depth_mm"]
    rclpy.init()
    node = rclpy.create_node("tcp_offset")
    state = RobotState(node, cfg["robot"]["state_topic"], config.link8_to_flange(cfg),
                       cfg["robot"]["state_timeout_s"])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        flange = state.flange_and_ee()[0]  # waits for the first robot state
        tcp = frames.tcp_position(flange, cfg["frames"]["flange_to_tcp_mm"])
        close = np.asarray(stud["close_mm"][:2], float)
        correction = tcp[:2] - close
        result = {"time": datetime.now().isoformat(timespec="seconds"), "run": record_name,
                  "stud_close_mm": sequences.rounded(close), "tcp_mm": sequences.rounded(tcp),
                  "tilt_deg": round(frames.tilt_deg(flange[:3, :3]), 2),
                  "correction_mm": sequences.rounded(correction),
                  "thread_in_iris_mm": round(float(tip_z - tcp[2]), 1),
                  "camera_tilt_in_flange_deg": cfg["frames"]["camera_tilt_in_flange_deg"],
                  "camera_rotation_about_z_deg": cfg["frames"]["camera_rotation_about_z_deg"],
                  # what target_correction added in that run (target - close-up stud)
                  "correction_in_run_mm": sequences.rounded(
                      np.asarray(stud["position_mm"][:2]) - close)}
        log_path = directory / "tcp_offset.yaml"
        history = (yaml.safe_load(log_path.read_text()) if log_path.exists() else None) or []
        history.append(result)
        log_path.write_text(yaml.safe_dump(history, sort_keys=False))
        print(f"stud (close-up, {record_name}): x {close[0]:.2f} y {close[1]:.2f} mm")
        print(f"TCP now: x {tcp[0]:.2f} y {tcp[1]:.2f} z {tcp[2]:.2f} mm, tilt "
              f"{result['tilt_deg']} deg")
        print(f"correction = TCP - stud: x {correction[0]:+.2f} y {correction[1]:+.2f} mm "
              f"(at y {close[1]:.0f}); appended to {log_path}")
        # 27.10.: two hand-centred measurements with the iris 3-10 mm above the tip differed by
        # 8 mm; centred by eye from above the tip is not repeatable
        depth = tip_z - tcp[2]
        print(f"thread in the iris: {depth:.1f} mm (TCP {tcp[2] - table_z:.1f} mm above the "
              f"table, tip at {tip_z - table_z:.0f} mm)")
        if depth < min_depth:
            print(f"WARNING: the thread is not {min_depth:.0f} mm inside the iris: guide the "
                  "gripper lower and measure again (by eye from above it is not repeatable)")
        print("all so far:")
        for entry in history:
            print(f"  {entry['time']}  y {entry['stud_close_mm'][1]:7.1f}  correction "
                  f"{entry['correction_mm']}")
    finally:
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
