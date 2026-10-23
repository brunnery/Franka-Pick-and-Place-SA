"""Go to the observation poses one after the other, e.g. to mark the part areas on the table.

  ros2 run sa_tasks goto_poses     # needs the robot (sa_robot) running; no gripper, no vision

Moves to observe_nut, then observe_stud (as in a normal run, same height), each after Enter.
At each pose: where the camera axis meets the table and where the TCP is above it (base frame).
"""
import logging
import threading

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, sequences
from sa_tasks.ros_interfaces import FrankaRobot

log = logging.getLogger("sa_tasks")

POSES = ("observe_nut", "observe_stud")


def main():
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    cfg = config.load_tasks()
    poses = config.Poses()
    table_z = poses.table_z()
    rclpy.init()
    node = rclpy.create_node("sa_tasks_goto_poses")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        robot = FrankaRobot(node, cfg)
        robot.set_mode(cfg["robot"]["modes"]["normal"])
        for name in POSES:
            input(f"Enter: move to {name} (Ctrl+C aborts) ")
            sequences.move_to(robot, sequences.observation_pose(poses, name, cfg), cfg)
            camera, distance = sequences.camera_and_distance(robot, cfg, table_z)
            on_table = camera[:3, 3] + distance * camera[:3, 2]
            flange = robot.flange_pose()
            tcp_xyz = flange[:3, :3] @ np.asarray(cfg["frames"]["flange_to_tcp_mm"]) \
                + flange[:3, 3]
            log.info("%s reached | camera axis on table x %.1f y %.1f mm | TCP x %.1f y %.1f "
                     "(%.1f mm above the table)", name, *on_table[:2], *tcp_xyz[:2],
                     tcp_xyz[2] - table_z)
        input("Enter: end (the arm stays where it is) ")
    except KeyboardInterrupt:
        log.info("aborted")
    finally:
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
