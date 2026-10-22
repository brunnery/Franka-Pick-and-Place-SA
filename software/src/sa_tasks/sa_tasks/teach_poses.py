"""Record the observation poses and the table height from the real robot.

  ros2 launch software/launch/robot_state.launch.py   # robot state only, no controller
  ros2 run sa_tasks teach_poses                       # second terminal

Guide the arm by hand (pilot buttons) and type a command:
  Enter          show the current pose
  save <name>    store the current flange pose (observe_nut, observe_stud)
  table          gripper bottom resting flat on the table: store its height
  list           show what is stored
  quit
Written to config/poses.yaml of sa_tasks at once.
"""
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, frames
from sa_tasks.ros_interfaces import RobotState


def main():
    cfg = config.load_tasks()
    poses = config.Poses()
    rclpy.init()
    node = rclpy.create_node("teach_poses")
    state = RobotState(node, cfg["robot"]["state_topic"], config.link8_to_flange(cfg),
                       cfg["robot"]["state_timeout_s"])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    print(__doc__.split("\n\n", 1)[1])
    try:
        while True:
            command = input("> ").strip().split()
            if command[:1] in (["quit"], ["q"]):
                break
            if command[:1] == ["list"]:
                print(f"table_z_mm: {poses.table_z_mm}")
                for name, entry in poses.poses.items():
                    print(f"{name}: {entry['position_mm']}")
                continue
            flange = state.flange_and_ee()[0]
            tcp = frames.tcp_position(flange, cfg["frames"]["flange_to_tcp_mm"])
            if not command:
                print(f"flange {flange[:3, 3].round(1).tolist()} mm, tilt "
                      f"{frames.tilt_deg(flange[:3, :3]):.1f} deg, TCP {tcp.round(1).tolist()} mm")
            elif command[0] == "table":
                poses.table_z_mm = float(tcp[2])
                poses.save()
                print(f"table z = {tcp[2]:.1f} mm saved")
            elif command[0] == "save" and len(command) == 2:
                poses.set(command[1], flange)
                poses.save()
                print(f"{command[1]} saved: {flange[:3, 3].round(1).tolist()} mm")
            else:
                print("unknown command")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()
