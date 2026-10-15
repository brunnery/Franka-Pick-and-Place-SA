"""Record the reference image of the empty work area (background for the nut detection).

  ros2 run sa_vision capture_reference [--params software/src/sa_vision/config/vision.yaml]
                                       [--frames 30] [--out <png>]

Before: clear the work area, move the robot to the observation pose and start the
camera with camera.launch.py (fixed exposure). Averages --frames raw frames to
suppress sensor noise and writes them to reference_image of vision.yaml.
Record it again whenever the lighting, the exposure or the observation pose changes.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from sa_vision.params import load_vision_config

DEFAULT_PARAMS = Path("software/src/sa_vision/config/vision.yaml")  # relative to the repo root
SPIN_TIMEOUT_S = 1.0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument("--frames", type=int, default=30, help="number of frames to average")
    parser.add_argument("--out", type=Path, help="override reference_image of vision.yaml")
    args, ros_args = parser.parse_known_args()
    out = args.out or Path(load_vision_config(args.params).reference_image)
    if out == Path("."):
        parser.error("reference_image is empty in vision.yaml, pass --out")

    rclpy.init(args=ros_args)
    node = rclpy.create_node("capture_reference")
    bridge = CvBridge()
    frames = []
    node.create_subscription(
        Image, "image_raw",
        lambda msg: frames.append(bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")),
        qos_profile_sensor_data)
    print(f"Averaging {args.frames} frames from image_raw ...")
    try:
        while rclpy.ok() and len(frames) < args.frames:
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    if len(frames) < args.frames:
        print(f"Aborted after {len(frames)} frames, nothing written.")
        return

    mean = np.mean(np.stack(frames[:args.frames]).astype(np.float32), axis=0)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), np.round(mean).astype(np.uint8))
    print(f"Reference image written to {out.resolve()}")


if __name__ == "__main__":
    main()
