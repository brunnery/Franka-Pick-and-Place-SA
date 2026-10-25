"""Motion test: the moves of a run without gripper and vision, with the jitter measured.

  ros2 run sa_tasks motion_test    # needs the robot (sa_robot) running

Per cycle and observation pose: the move there, straight down to low_tcp_height_mm above
the table and back up (as the descent to the nut). During each move the measured flange
position and control_command_success_rate are sampled; the record holds per move the
vibration RMS (analysis.vibration_rms_mm) and the lowest success rate, plus the controller
gains of controllers.yaml (what sa_robot loaded, if it was restarted after the last edit).
Starts after Enter, then runs on its own; Ctrl+C aborts. Record in log_dir.
"""
import logging
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor

from sa_tasks import analysis, config, frames, sequences
from sa_tasks.ros_interfaces import M_TO_MM, FrankaRobot
from sa_tasks.trajectory import LEAD_Z

log = logging.getLogger("sa_tasks")

CONTROLLER_KEYS = ("damping", "modes", "filter_params", "delta_tau_max", "saturation")


class Sampler:
    """Polls the latest robot state in a thread: (robot time, x, y, z of the EE in mm,
    success rate)."""

    def __init__(self, robot: FrankaRobot, period_s: float):
        self.robot, self.period = robot, period_s
        self.samples, self.running, self.thread = [], False, None

    def start(self) -> None:
        self.samples, self.running = [], True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> np.ndarray:
        self.running = False
        self.thread.join()
        return np.array(self.samples).reshape(-1, 5)

    def _run(self) -> None:
        last = None
        while self.running:
            msg = self.robot.state.msg
            if msg is not None and msg is not last:
                p = msg.o_t_ee.pose.position  # time: robot clock, not when it was polled
                self.samples.append((msg.time, p.x * M_TO_MM, p.y * M_TO_MM,
                                     p.z * M_TO_MM, msg.control_command_success_rate))
                last = msg
            time.sleep(self.period)


def controller_gains() -> dict:
    path = (Path(get_package_share_directory("cartesian_impedance_control")) / "launch"
            / "controllers.yaml")
    params = yaml.safe_load(path.read_text())["/**"]["cartesian_impedance_controller"]
    params = params["ros__parameters"]
    return {key: params[key] for key in CONTROLLER_KEYS if key in params}


def main():
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    cfg = config.load_tasks()
    test = cfg["motion_test"]
    if test["max_speed_mm_s"]:  # only for this test, the runs keep robot.motion
        cfg["robot"]["motion"]["max_speed_mm_s"] = test["max_speed_mm_s"]
    poses = config.Poses()
    table_z = poses.table_z()
    tcp_offset = cfg["frames"]["flange_to_tcp_mm"]
    rclpy.init()
    node = rclpy.create_node("sa_tasks_motion_test")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    motion = cfg["robot"]["motion"]
    record = {"controller": controller_gains(), "max_speed_mm_s": motion["max_speed_mm_s"],
              "max_speed_z_mm_s": motion["max_speed_z_mm_s"], "moves": []}
    try:
        robot = FrankaRobot(node, cfg)
        robot.set_mode(cfg["robot"]["modes"]["normal"])
        sampler = Sampler(robot, test["sample_period_s"])

        def measured(label: str, goal: np.ndarray, lead=None) -> None:
            sampler.start()
            started = time.monotonic()
            sequences.move_to(robot, goal, cfg, lead=lead)
            samples = sampler.stop()
            move = {"move": label, "duration_s": round(time.monotonic() - started, 2),
                    "vibration_rms_mm": analysis.vibration_rms_mm(
                        samples[:, 0], samples[:, 1:4], test["vibration_window_s"]),
                    "min_success_rate": round(float(samples[:, 4].min()), 3)
                    if len(samples) else None}
            record["moves"].append(move)
            log.info("%-22s %5.2f s | vibration x %.3f y %.3f z %.3f mm | success rate min %s",
                     label, move["duration_s"], *move["vibration_rms_mm"],
                     move["min_success_rate"])

        input("Enter: start (the arm moves on its own, Ctrl+C aborts) ")
        for cycle in range(1, test["cycles"] + 1):
            for name in test["poses"]:
                observe = sequences.observation_pose(poses, name, cfg)
                measured(f"{cycle} to {name}", observe)
                tcp = frames.tcp_position(observe, tcp_offset)
                low = frames.flange_for_tcp([tcp[0], tcp[1], table_z + test["low_tcp_height_mm"]],
                                            config.rotation_down(cfg), tcp_offset)
                measured(f"{cycle} down at {name}", low)
                measured(f"{cycle} up at {name}", observe, LEAD_Z)
        moves = record["moves"]
        for kind in ("to", "down", "up"):
            rms = [m["vibration_rms_mm"] for m in moves if m["move"].split()[1] == kind]
            if rms:
                log.info("mean %-4s vibration x %.3f y %.3f z %.3f mm", kind, *np.mean(rms, axis=0))
    except KeyboardInterrupt:
        log.info("aborted")
    except Exception as error:  # noqa: BLE001 - logged, the record is still written
        log.error("%s", error)
        record["failed"] = str(error)
    finally:
        directory = config.PACKAGE_DIR.parents[2] / cfg["log_dir"]
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{datetime.now():%Y%m%d-%H%M%S}_motion_test.yaml"
        path.write_text(yaml.safe_dump(record, sort_keys=False))
        log.info("record: %s", path)
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
