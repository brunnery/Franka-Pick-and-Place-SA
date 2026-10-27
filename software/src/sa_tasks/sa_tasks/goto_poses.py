"""Go to the observation poses one after the other, e.g. to mark the part areas on the table.

  ros2 run sa_tasks goto_poses          # needs the robot (sa_robot) running; no gripper, no vision
  ros2 run sa_tasks goto_poses --mark   # also down to the table: centre and corners of each area

Moves to observe_nut, then observe_stud (as in a normal run, same height), each after Enter.
At each pose: where the camera axis meets the table and where the TCP is above it (base frame).
--mark (27.10.): then, each after Enter, the TCP (centre of the bottom plate) goes to
goto_poses.mark_height_mm above the table at the centre of the area (where the camera axis meets
the table) and at its four corners (run_map.area_half_size_mm), and back up. The table must
be clear.
"""
import argparse
import logging
import threading

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, frames, sequences
from sa_tasks.ros_interfaces import FrankaRobot
from sa_tasks.trajectory import LEAD_XY, LEAD_Z

log = logging.getLogger("sa_tasks")

POSES = ("observe_nut", "observe_stud")


def mark_area(robot: FrankaRobot, cfg: dict, centre_xy, table_z: float) -> None:
    """TCP just above the table at the centre and the corners of an area, each after Enter."""
    hx, hy = cfg["run_map"]["area_half_size_mm"]
    height = table_z + cfg["goto_poses"]["mark_height_mm"]
    rotation, tcp_offset = config.rotation_down(cfg), cfg["frames"]["flange_to_tcp_mm"]
    # seen from the robot base: far = +x (away from the base), left = +y
    points = [("centre", 0.0, 0.0), ("far left corner", hx, hy), ("far right corner", hx, -hy),
              ("near right corner", -hx, -hy), ("near left corner", -hx, hy)]
    for i, (label, dx, dy) in enumerate(points):
        input(f"Enter: TCP to the {label} of the area (table clear?) ")
        goal = frames.flange_for_tcp([centre_xy[0] + dx, centre_xy[1] + dy, height], rotation,
                                     tcp_offset)
        sequences.move_to(robot, goal, cfg, lead=LEAD_XY if i == 0 else None)
        log.info("%s: TCP over x %.1f y %.1f mm (mark it below the centre of the plate)", label,
                 centre_xy[0] + dx, centre_xy[1] + dy)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mark", action="store_true",
                        help="also go down to the centre and the corners of each area")
    args, _ = parser.parse_known_args()
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
            if args.mark:
                observe = sequences.observation_pose(poses, name, cfg)
                mark_area(robot, cfg, on_table[:2], table_z)
                input(f"Enter: back up to {name} ")
                sequences.move_to(robot, observe, cfg, lead=LEAD_Z)
        input("Enter: end (the arm stays where it is) ")
    except KeyboardInterrupt:
        log.info("aborted")
    finally:
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
