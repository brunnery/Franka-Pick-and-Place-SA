"""Motion test: the moves of a run without gripper and vision, with the jitter measured.

  ros2 run sa_tasks motion_test    # needs the robot (sa_robot) running

motion_test.mode "run" (27.10.): the moves of a full run, rebuilt from the newest full run
record (nut and stud as found there), with the same move options (first look while creeping,
lift lead fraction, z leading up). The low points (pick, approach over the stud) stay at
low_tcp_height_mm above the table, so nut and stud may stay where they are.
motion_test.mode "poses": per cycle and observation pose: the move there, straight down to
low_tcp_height_mm above the table and back up (as the descent to the nut). During each move the measured flange
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
from sa_tasks.trajectory import LEAD_XY, LEAD_Z

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


def newest_full_run(directory: Path) -> tuple:
    """(record name, nut xy of the first look, stud xy of the first look) of the newest full
    run record that got that far."""
    for path in sorted(directory.glob("*_full.yaml"), reverse=True):
        try:
            record = yaml.safe_load(path.read_text()) or {}
        except yaml.YAMLError:
            continue
        nut = (record.get("pick") or {}).get("nut_coarse_mm")
        stud = ((record.get("place") or {}).get("stud") or {}).get("observed_mm")
        if nut and stud:
            return path.name, nut[:2], stud[:2]
    raise RuntimeError(f"no full run record with nut and stud in {directory}")


def run_moves(cfg: dict, poses: config.Poses, nut_xy, stud_xy, low_z: float) -> list:
    """(label, goal, move_to options) of the moves of a full run (sequences.pick and
    place_and_screw), the low points at low_z (TCP)."""
    rotation, tcp_offset = config.rotation_down(cfg), cfg["frames"]["flange_to_tcp_mm"]
    table_z = poses.table_z()
    nut_plane = table_z + cfg["objects"]["nut"]["height_mm"]
    stud_plane = table_z + cfg["objects"]["stud"]["base_height_mm"]
    stud_close = sequences.close_up_pose(cfg, stud_xy, stud_plane)
    return [
        ("to observe_nut", sequences.observation_pose(poses, "observe_nut", cfg),
         {"rough": True}),
        ("down to nut close-up", sequences.close_up_pose(cfg, nut_xy, nut_plane), {}),
        ("down to nut", frames.flange_for_tcp([*nut_xy, low_z], rotation, tcp_offset),
         {"lead": LEAD_XY}),
        ("lift to observe_stud", sequences.observation_pose(poses, "observe_stud", cfg),
         {"rough": True, "lead_fraction": cfg["place"]["lift_lead_fraction"],
          "max_speed_z_mm_s": cfg["place"]["lift_max_speed_z_mm_s"]}),
        ("down to stud close-up", stud_close, {}),
        ("down to stud approach", frames.flange_for_tcp([*stud_xy, low_z], rotation,
                                                        tcp_offset), {"precise": True}),
        ("up to stud close-up", stud_close, {"lead": LEAD_Z}),
    ]


def peak_speeds(samples: np.ndarray) -> list:
    """Highest measured speed in xy and in z [mm/s] (samples: time s, x, y, z mm, ...)."""
    if len(samples) < 3:
        return [0.0, 0.0]
    dt = np.diff(samples[:, 0])
    ok = dt > 0
    v = np.diff(samples[:, 1:4], axis=0)[ok] / dt[ok, None]
    return [round(float(np.max(np.linalg.norm(v[:, :2], axis=1))), 1),
            round(float(np.max(np.abs(v[:, 2]))), 1)]


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

        def measured(cycle: int, label: str, goal: np.ndarray, **options) -> None:
            sampler.start()
            started = time.monotonic()
            sequences.move_to(robot, goal, cfg, **options)
            samples = sampler.stop()
            move = {"cycle": cycle, "move": label,
                    "duration_s": round(time.monotonic() - started, 2),
                    "vibration_rms_mm": analysis.vibration_rms_mm(
                        samples[:, 0], samples[:, 1:4], test["vibration_window_s"]),
                    "peak_speed_xy_z_mm_s": peak_speeds(samples),
                    "min_success_rate": round(float(samples[:, 4].min()), 3)
                    if len(samples) else None}
            record["moves"].append(move)
            log.info("%d %-22s %5.2f s | vibration x %.3f y %.3f z %.3f mm | peak xy %.0f "
                     "z %.0f mm/s | success rate min %s", cycle, label, move["duration_s"],
                     *move["vibration_rms_mm"], *move["peak_speed_xy_z_mm_s"],
                     move["min_success_rate"])

        low_z = table_z + test["low_tcp_height_mm"]
        if test["mode"] == "run":
            name, nut_xy, stud_xy = newest_full_run(
                config.PACKAGE_DIR.parents[2] / cfg["log_dir"])
            record["from_record"] = name
            log.info("moves of a full run, nut and stud as in %s", name)
            moves = run_moves(cfg, poses, nut_xy, stud_xy, low_z)
        else:
            moves = []
            for name in test["poses"]:
                observe = sequences.observation_pose(poses, name, cfg)
                tcp = frames.tcp_position(observe, tcp_offset)
                low = frames.flange_for_tcp([tcp[0], tcp[1], low_z], config.rotation_down(cfg),
                                            tcp_offset)
                moves += [(f"to {name}", observe, {}), (f"down at {name}", low, {}),
                          (f"up at {name}", observe, {"lead": LEAD_Z})]
        input("Enter: start (the arm moves on its own, Ctrl+C aborts) ")
        for cycle in range(1, test["cycles"] + 1):
            for label, goal, options in moves:
                measured(cycle, label, goal, **options)
        for label in dict.fromkeys(m["move"] for m in record["moves"]):
            rms = [m["vibration_rms_mm"] for m in record["moves"] if m["move"] == label]
            log.info("mean %-22s vibration x %.3f y %.3f z %.3f mm", label,
                     *np.mean(rms, axis=0))
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
