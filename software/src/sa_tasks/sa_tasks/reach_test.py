"""Reach test: where does the arm really stop, and why? No gripper, no vision.

  ros2 run sa_tasks reach_test     # needs the robot (sa_robot) running

Moves the TCP over a small grid around each centre of reach_test in tasks.yaml, at a safe
height above the table. At every point: the plain move (remaining error as in a normal run),
the external force the robot estimates (what holds the arm off the target), the smallest
joint-limit margin, then the fine positioning (THEORY 5) and its result. Record in log_dir.
"""
import logging
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import config, frames, sequences
from sa_tasks.ros_interfaces import FrankaRobot
from sa_tasks.trajectory import LEAD_Z

log = logging.getLogger("sa_tasks")

# FR3 joint position limits in rad (Franka FR3 datasheet), q1..q7
Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
Q_MAX = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])


def snapshot(robot: FrankaRobot, goal: np.ndarray) -> dict:
    msg = robot.state.latest()
    q = np.asarray(msg.measured_joint_state.position[:7])
    margin = np.minimum(q - Q_MIN, Q_MAX - q)
    force = msg.o_f_ext_hat_k.wrench.force
    return {"error_mm": sequences.rounded(robot.flange_pose()[:3, 3] - goal[:3, 3]),
            "force_ext_n": sequences.rounded([force.x, force.y, force.z]),
            "q_rad": sequences.rounded(q),
            "closest_limit": f"q{int(np.argmin(margin)) + 1} {np.degrees(margin.min()):.1f} deg"}


def main():
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    cfg = config.load_tasks()
    test = cfg["reach_test"]
    table_z = config.Poses().table_z()
    rclpy.init()
    node = rclpy.create_node("sa_tasks_reach_test")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    points = []
    try:
        robot = FrankaRobot(node, cfg)
        robot.set_mode(cfg["robot"]["modes"]["normal"])
        step = test["grid_mm"]
        for centre in test["centres_mm"]:
            for dx, dy in ((0, 0), (step, 0), (-step, 0), (0, step), (0, -step)):
                xy = [centre[0] + dx, centre[1] + dy]
                goal = frames.flange_for_tcp([*xy, table_z + test["height_above_table_mm"]],
                                             config.rotation_down(cfg),
                                             cfg["frames"]["flange_to_tcp_mm"])
                sequences.move_to(robot, goal, cfg, lead=LEAD_Z)
                plain = snapshot(robot, goal)
                time.sleep(test["hold_s"])
                held = snapshot(robot, goal)
                reference = sequences.move_to(robot, goal, cfg, precise=True)
                fine = snapshot(robot, goal)
                fine["reference_shift_mm"] = sequences.rounded(reference[:2, 3] - goal[:2, 3])
                points.append({"tcp_xy_mm": xy, "plain": plain, f"after_{test['hold_s']}_s": held,
                               "fine": fine})
                log.info("x %6.1f y %6.1f | plain err %s F_ext %s | fine err %s shift %s | %s",
                         *xy, plain["error_mm"], plain["force_ext_n"], fine["error_mm"],
                         fine["reference_shift_mm"], fine["closest_limit"])
    except KeyboardInterrupt:
        log.info("aborted")
    except Exception as error:  # noqa: BLE001 - logged, the record is still written
        log.error("%s", error)
        points.append({"failed": str(error)})
    finally:
        directory = config.PACKAGE_DIR.parents[2] / cfg["log_dir"]
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{datetime.now():%Y%m%d-%H%M%S}_reach_test.yaml"
        path.write_text(yaml.safe_dump({"points": points}, sort_keys=False))
        log.info("record: %s", path)
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
