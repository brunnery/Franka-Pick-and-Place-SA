"""Entry points of step 1 and step 2 (usually started by software/launch/task.launch.py).

  ros2 run sa_tasks pick           # step 1: nut -> gripper, back to the observation pose
  ros2 run sa_tasks place_screw    # step 2: stud -> place, screw, release, back up
                                   #   --approach-only: stop above the thread (no gripper)
  ros2 run sa_tasks full           # step 1 + 2 in one process (the gripper watchdog lets go
                                   # 2 s after a process ends)
  --robot mock: dry run without the robot, vision still real.

Writes one YAML record per run to log_dir of tasks.yaml, with the camera and vision debug
image of every detection next to it (<record>_<n>_<label>_<raw|debug>.png).
"""
import argparse
import logging
import threading
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, sequences, serial_gripper
from sa_tasks.interfaces import MockGripper, MockRobot
from sa_tasks.ros_interfaces import FrankaRobot, RosVision

log = logging.getLogger("sa_tasks")


def run(step: str) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", choices=("franka", "mock"), default="franka")
    parser.add_argument("--approach-only", action="store_true")
    args, ros_args = parser.parse_known_args()
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    cfg = config.load_tasks()
    poses = config.Poses()

    rclpy.init(args=ros_args)
    node = rclpy.create_node(f"sa_tasks_{step}")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    started = time.monotonic()
    record = {"step": step, "started": datetime.now().isoformat()}
    robot = gripper = vision = None
    try:
        if args.robot == "mock":  # the robot "is" at the observation pose
            start = "observe_stud" if step == "place_and_screw" else "observe_nut"
            robot = MockRobot(poses.get(start), cfg["frames"]["flange_to_tcp_mm"][2])
        else:
            robot = FrankaRobot(node, cfg)
        gripper = MockGripper() if args.approach_only else serial_gripper.connect(cfg["gripper"])
        vision = RosVision(node, cfg)
        if step in ("pick", "full"):
            record["pick"] = {}
            # full: straight from the table to observe_stud, no stop at observe_nut
            sequences.pick(robot, gripper, vision, cfg, poses, record["pick"],
                           back_up=step == "pick")
        if step in ("place_and_screw", "full"):
            record["place"] = {}
            sequences.place_and_screw(robot, gripper, vision, cfg, poses, record["place"],
                                      args.approach_only)
        record["result"] = "ok"
        log.info("done in %.1f s", time.monotonic() - started)
    except KeyboardInterrupt:
        record["result"] = "aborted by user"
    except Exception as error:  # noqa: BLE001 - every failure ends up in the record
        record["result"] = f"failed: {error}"
        log.error("%s", error)
    finally:
        record["duration_s"] = round(time.monotonic() - started, 2)
        if gripper is not None:
            gripper.shutdown()
        record["arrival_errors_mm"] = getattr(robot, "arrivals", [])
        directory = config.PACKAGE_DIR.parents[2] / cfg["log_dir"]
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{datetime.now():%Y%m%d-%H%M%S}_{step}.yaml"
        path.write_text(yaml.safe_dump(record, sort_keys=False))
        log.info("record: %s", path)
        if vision is not None:  # camera + vision images of each detection, next to the record
            vision.save_snapshots(path.with_suffix(""))
        # Stop the spinning thread before the context goes, else rclcpp aborts
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


def main_pick():
    run("pick")


def main_place():
    run("place_and_screw")


def main_full():
    run("full")
