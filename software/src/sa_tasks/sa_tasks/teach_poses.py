"""Record the hardcoded poses and the table height from the real robot.

  ros2 launch software/launch/robot_state.launch.py   # robot state only, no controller
  ros2 run sa_tasks teach_poses                       # second terminal

Guide the arm by hand (pilot buttons) and type a command:
  Enter               show the current pose
  save <name>         store the current flange pose, e.g. save observe_nut / save observe_stud
  table               gripper bottom resting flat on the table: store its height
  list                show what is stored
  quit
The poses are written to config/poses.yaml of sa_tasks at once.
"""
import argparse
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo

from sa_tasks import config, frames
from sa_tasks.ros_interfaces import RobotState
from sa_vision import geometry

REQUIRED = ("observe_nut", "observe_stud")


def describe(t_base_flange: np.ndarray, cfg: dict, poses: config.Poses, k) -> str:
    rotation_down = config.rotation_down(cfg)
    p = t_base_flange[:3, 3]
    tcp = frames.tcp_position(t_base_flange, cfg["frames"]["flange_to_tcp_mm"])
    lines = [f"flange x {p[0]:7.1f}  y {p[1]:7.1f}  z {p[2]:7.1f} mm | "
             f"tilt {frames.tilt_deg(t_base_flange[:3, :3]):4.1f} deg, "
             f"{frames.rotation_angle_deg(t_base_flange[:3, :3], rotation_down):5.1f} deg "
             f"from the commanded orientation"]
    if poses.table_z_mm is not None:
        camera = t_base_flange @ config.camera_in_flange(cfg)
        d = frames.distance_along_axis(camera, poses.table_z_mm)
        line = f"TCP {tcp[2] - poses.table_z_mm:6.1f} mm above the table, camera d {d:.0f} mm"
        if k is not None:
            width, height = frames.footprint_mm(d, k, (int(2 * k[0, 2]), int(2 * k[1, 2])))
            axis = frames.object_in_base(k[0, 2], k[1, 2], k, camera, poses.table_z_mm)
            line += (f", image covers about {width:.0f} x {height:.0f} mm around "
                     f"x {axis[0]:.0f} y {axis[1]:.0f}")
        lines.append(line)
    else:
        lines.append(f"TCP z {tcp[2]:.1f} mm (table height not taught yet: 'table')")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=config.DEFAULT_TASKS)
    parser.add_argument("--poses", type=Path, default=config.DEFAULT_POSES)
    args, ros_args = parser.parse_known_args()
    cfg = config.load_tasks(args.config)
    poses = config.Poses(args.poses)

    rclpy.init(args=ros_args)
    node = rclpy.create_node("teach_poses")
    state = RobotState(node, cfg["robot"]["state_topic"])
    camera = {"k": None}
    node.create_subscription(
        CameraInfo, cfg["vision"]["camera_info_topic"],
        lambda msg: camera.update(k=geometry.camera_matrix(msg.k)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    timeout = cfg["robot"]["state_timeout_s"]
    print(__doc__.split("\n\n", 1)[1])
    print(f"Writing to {poses.path}")
    try:
        while True:
            command = input("> ").strip().split()
            if command and command[0] in ("quit", "q", "exit"):
                break
            if command and command[0] == "list":
                print(f"table_z_mm: {poses.table_z_mm}")
                for name, entry in poses.poses.items():
                    print(f"{name}: {entry['position_mm'] if entry else 'not taught'}")
                continue
            current = state.flange_and_ee(timeout)[0]
            if not command:
                print(describe(current, cfg, poses, camera["k"]))
            elif command[0] == "table":
                tcp = frames.tcp_position(current, cfg["frames"]["flange_to_tcp_mm"])
                poses.table_z_mm = float(tcp[2])
                poses.save()
                print(f"table z = {tcp[2]:.1f} mm saved (the gripper bottom must rest flat "
                      "on the table)")
            elif command[0] == "save" and len(command) == 2:
                tilt = frames.tilt_deg(current[:3, :3])
                if tilt > cfg["robot"]["max_taught_tilt_deg"]:
                    print(f"not saved: tool tilted by {tilt:.1f} deg (max "
                          f"{cfg['robot']['max_taught_tilt_deg']} deg), straighten it first")
                    continue
                poses.set(command[1], current)
                poses.save()
                print(f"{command[1]} saved:\n{describe(current, cfg, poses, camera['k'])}")
                if command[1] not in REQUIRED:
                    print(f"note: the sequences use {REQUIRED}")
            else:
                print("unknown command")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        # Stop the spinning thread before the context goes, else rclcpp aborts
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
