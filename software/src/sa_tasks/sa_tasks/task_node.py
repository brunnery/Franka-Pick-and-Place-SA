"""Entry points of step 1 and step 2.

  ros2 run sa_tasks pick           # step 1: nut -> gripper, back to the observation pose
  ros2 run sa_tasks place_screw    # step 2: stud -> place, screw, release, back up
  options: --robot mock (dry run, vision still real), --yes (no confirmation)

Needs: sa_vision (camera + vision_node) and, with the real robot, the impedance
controller (sa_robot). Writes one YAML record per run to log_dir of tasks.yaml.
"""
import argparse
import logging
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, sequences
from sa_tasks.interfaces import MockGripper, MockRobot
from sa_tasks.ros_interfaces import FrankaRobot, RosVision

REPO_ROOT = config.PACKAGE_DIR.parents[2]
ROBOT_BACKENDS = ("franka", "mock")


def make_gripper(cfg: dict):
    backend = cfg["gripper"]["backend"]
    if backend == "mock":
        return MockGripper()
    raise ValueError(f"gripper backend '{backend}' not implemented yet (only 'mock')")


def make_robot(node, cfg: dict, backend: str, poses: config.Poses, start_pose: str):
    if backend not in ROBOT_BACKENDS:
        raise ValueError(f"robot backend must be one of {ROBOT_BACKENDS}, not '{backend}'")
    if backend == "mock":
        # Dry run: the robot "is" at the observation pose, the camera on its stand looks
        # at the real scene
        return MockRobot(poses.get(start_pose), cfg["frames"]["flange_to_tcp_mm"][2])
    robot = FrankaRobot(node, cfg)
    robot.check_controller()
    return robot


def write_record(cfg: dict, record: dict) -> Path:
    directory = REPO_ROOT / cfg["log_dir"]
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{datetime.now():%Y%m%d-%H%M%S}_{record['step']}.yaml"
    path.write_text(yaml.safe_dump(record, sort_keys=False))
    return path


def run(step: str) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=config.DEFAULT_TASKS)
    parser.add_argument("--poses", type=Path, default=config.DEFAULT_POSES)
    parser.add_argument("--robot", choices=ROBOT_BACKENDS, help="override robot.backend")
    parser.add_argument("--yes", action="store_true", help="start without confirmation")
    if step == "pick":
        parser.add_argument("--locate-only", action="store_true",
                            help="observation pose + detection only, no descent")
    args, ros_args = parser.parse_known_args()
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    log = logging.getLogger("sa_tasks")

    cfg = config.load_tasks(args.config)
    poses = config.Poses(args.poses)
    backend = args.robot or cfg["robot"]["backend"]
    start_pose = "observe_nut" if step == "pick" else "observe_stud"

    rclpy.init(args=ros_args)
    node = rclpy.create_node(f"sa_tasks_{step}")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    started = time.monotonic()
    record = {"step": step, "robot": backend, "started": datetime.now().isoformat()}
    robot = None
    try:
        robot = make_robot(node, cfg, backend, poses, start_pose)
        gripper = make_gripper(cfg)
        vision = RosVision(node, cfg)
        table = poses.table_z()
        observe = poses.get(start_pose)
        log.info("robot %s, gripper %s, table z %.1f mm, %s at %s mm", backend,
                 cfg["gripper"]["backend"], table, start_pose,
                 [round(v, 1) for v in observe[:3, 3]])
        if not args.yes:
            input(f"Step '{step}': the robot will move. Check first: after the controller "
                  "activation the camera faces away from the robot base (+x). "
                  "Enter = start, Ctrl-C = abort ")
        if step == "pick":
            sequences.pick(robot, gripper, vision, cfg, poses, locate_only=args.locate_only,
                           record=record)
        else:
            record.update(sequences.place_and_screw(robot, gripper, vision, cfg, poses))
        record["result"] = "ok"
        log.info("done in %.1f s", time.monotonic() - started)
    except KeyboardInterrupt:
        record["result"] = "aborted by user"
        log.warning("aborted: the arm holds the last reference")
    except Exception as error:  # noqa: BLE001 - every failure ends up in the record
        record["result"] = f"failed: {error}"
        log.error("%s", error)
    finally:
        record["duration_s"] = round(time.monotonic() - started, 2)
        if robot is not None and getattr(robot, "arrivals", None):
            record["arrival_errors_mm"] = robot.arrivals
        log.info("record: %s", write_record(cfg, record))
        # Stop the spinning thread before the context goes, else rclcpp aborts
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


def main_pick():
    run("pick")


def main_place():
    run("place_and_screw")


if __name__ == "__main__":
    main_pick()
