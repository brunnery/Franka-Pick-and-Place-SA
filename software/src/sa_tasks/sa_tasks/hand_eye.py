"""Camera-in-flange calibration from a ChArUco board flat on the table (THEORY 1.5).

  ros2 run sa_tasks hand_eye       # needs the robot (sa_robot) and the camera running

Place the board (ros2 run sa_tasks charuco_board) flat on the table, centred in the image at
hand_eye.centre_pose. After Enter the flange moves over the grid of hand_eye.offsets_*_mm
(always the fixed orientation, translations only), takes an image at each point, finds the
board (solvePnP via the ChArUco corners) and fits the rotation of the camera in the flange
(hand_eye_solver). Prints it as camera_tilt_in_flange_deg (x, y) plus the rotation about z,
next to the current value; nothing is written to tasks.yaml. Record and images in log_dir.
"""
import itertools
import logging
import threading
import time
from datetime import datetime
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from sa_tasks import config, hand_eye_solver, sequences
from sa_tasks.charuco_board import make_board
from sa_tasks.ros_interfaces import FrankaRobot, image_to_bgr

log = logging.getLogger("sa_tasks")


class Camera:
    """Latest image and intrinsics; image_after(t) waits for an image taken after t."""

    def __init__(self, node, cfg: dict):
        self.node = node
        self.image: Optional[Image] = None
        self.info: Optional[CameraInfo] = None
        node.create_subscription(Image, cfg["vision"]["raw_image_topic"],
                                 lambda m: setattr(self, "image", m), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, cfg["vision"]["camera_info_topic"],
                                 lambda m: setattr(self, "info", m), 10)

    def image_after(self, stamp_ns: int, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg, info = self.image, self.info
            if msg is not None and info is not None and \
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec >= stamp_ns:
                return image_to_bgr(msg), np.reshape(info.k, (3, 3)), np.asarray(info.d, float)
            time.sleep(0.01)
        raise RuntimeError(f"no camera image within {timeout_s} s (camera running?)")


def board_in_camera(image: np.ndarray, board, k: np.ndarray, d: np.ndarray,
                    min_corners: int) -> Optional[tuple]:
    """(R_C_board, origin of the board in the camera frame in mm, number of corners) or None."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    markers, ids, _ = cv2.aruco.detectMarkers(gray, board.dictionary)
    if ids is None or len(ids) == 0:
        return None
    count, corners, corner_ids = cv2.aruco.interpolateCornersCharuco(markers, ids, gray, board,
                                                                     cameraMatrix=k, distCoeffs=d)
    if count is None or count < min_corners:
        return None
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(corners, corner_ids, board, k, d,
                                                         None, None)
    if not ok:
        return None
    return cv2.Rodrigues(rvec)[0], tvec.reshape(3), int(count)


def main():
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    cfg = config.load_tasks()
    he = cfg["hand_eye"]
    poses = config.Poses()
    board = make_board(he["board"])
    t_fc = config.camera_in_flange(cfg)
    r_cad = np.asarray(cfg["frames"]["camera_rotation_in_flange"], float)
    rclpy.init()
    node = rclpy.create_node("sa_tasks_hand_eye")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    stamp = f"{datetime.now():%Y%m%d-%H%M%S}"
    directory = config.PACKAGE_DIR.parents[2] / cfg["log_dir"]
    directory.mkdir(parents=True, exist_ok=True)
    record = {"board": he["board"], "t_flange_camera_mm": sequences.rounded(t_fc[:3, 3]),
              "camera_tilt_in_flange_deg_now": cfg["frames"]["camera_tilt_in_flange_deg"],
              "views": []}
    try:
        robot = FrankaRobot(node, cfg)
        camera = Camera(node, cfg)
        robot.set_mode(cfg["robot"]["modes"]["normal"])
        centre = sequences.observation_pose(poses, he["centre_pose"], cfg)
        input(f"Board flat on the table, centred in the image at {he['centre_pose']}? "
              "Enter: start (the arm moves on its own, Ctrl+C aborts) ")
        grid = list(itertools.product(he["offsets_x_mm"], he["offsets_y_mm"], he["offsets_z_mm"]))
        for number, offset in enumerate(grid, 1):
            goal = centre.copy()
            goal[:3, 3] += offset
            sequences.move_to(robot, goal, cfg)
            time.sleep(he["settle_s"])
            flange = robot.flange_pose()
            image, k, d = camera.image_after(node.get_clock().now().nanoseconds,
                                             he["image_timeout_s"])
            found = board_in_camera(image, board, k, d, he["min_corners"])
            cv2.imwrite(str(directory / f"{stamp}_hand_eye_{number:02d}.png"), image)
            view = {"offset_mm": list(offset), "flange_mm": sequences.rounded(flange[:3, 3]),
                    "r_base_flange": flange[:3, :3].round(6).tolist(), "found": found is not None}
            if found is not None:
                view.update(corners=found[2], board_in_camera_mm=sequences.rounded(found[1]),
                            r_camera_board=found[0].round(6).tolist())
            record["views"].append(view)
            log.info("%2d/%d offset %s: %s", number, len(grid), offset,
                     f"{found[2]} corners, board at {np.round(found[1], 1)} mm" if found
                     else "board NOT found")
        sequences.move_to(robot, centre, cfg)
        used = [v for v in record["views"] if v["found"]]
        if len(used) < 6:
            raise RuntimeError(f"board found in only {len(used)} images, need at least 6")
        r_fc, board_base, residuals = hand_eye_solver.camera_rotation(
            [v["r_base_flange"] for v in used], [v["flange_mm"] for v in used],
            [v["board_in_camera_mm"] for v in used], t_fc[:3, 3], r_cad,
            he["solver_iterations"])
        tilt = hand_eye_solver.xyz_angles_deg(r_fc @ r_cad.T)
        # Check: the board lies flat, so its normal should be vertical in the base frame
        normals = [np.degrees(np.arccos(abs((np.asarray(v["r_base_flange"]) @ r_fc
                                              @ np.asarray(v["r_camera_board"]))[2, 2])))
                   for v in used]
        record["result"] = {
            "camera_tilt_in_flange_deg": [round(float(tilt[0]), 3), round(float(tilt[1]), 3)],
            "rotation_about_z_deg": round(float(tilt[2]), 3),
            "board_origin_in_base_mm": sequences.rounded(board_base),
            "residual_mm": {"rms": round(float(np.sqrt(np.mean(residuals ** 2))), 3),
                            "max": round(float(residuals.max()), 3)},
            "board_normal_from_vertical_deg": {"mean": round(float(np.mean(normals)), 3),
                                               "max": round(float(np.max(normals)), 3)},
            "views_used": len(used)}
        result = record["result"]
        log.info("camera_tilt_in_flange_deg: [%.2f, %.2f] (now %s), rotation about z %.2f deg",
                 *result["camera_tilt_in_flange_deg"], cfg["frames"]["camera_tilt_in_flange_deg"],
                 result["rotation_about_z_deg"])
        log.info("fit: residual RMS %.2f mm (max %.2f) over %d views; board normal %.2f deg from "
                 "vertical (max %.2f)", result["residual_mm"]["rms"], result["residual_mm"]["max"],
                 len(used), result["board_normal_from_vertical_deg"]["mean"],
                 result["board_normal_from_vertical_deg"]["max"])
    except KeyboardInterrupt:
        log.info("aborted")
    except Exception as error:  # noqa: BLE001 - logged, the record is still written
        log.error("%s", error)
        record["failed"] = str(error)
    finally:
        path = directory / f"{stamp}_hand_eye.yaml"
        path.write_text(yaml.safe_dump(record, sort_keys=False))
        log.info("record: %s", path)
        executor.shutdown()
        spinner.join(timeout=1.0)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
