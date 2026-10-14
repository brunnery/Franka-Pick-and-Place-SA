"""Save raw camera frames for offline tuning of the detection.

  ros2 run sa_vision capture_images --out data/images --prefix nut

Shows a live preview with the principal point. SPACE saves the current raw
frame as PNG (lossless), q or ESC quits. Run from a normal terminal (GUI).
"""
import argparse
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

WINDOW = "capture (SPACE = save, q = quit)"
PREVIEW_SCALE = 0.75
CROSS_COLOUR = (0, 220, 255)
CROSS_SIZE = 40
KEY_SAVE = ord(" ")
KEYS_QUIT = (ord("q"), 27)
KEY_WAIT_MS = 10


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="data/images", help="output directory")
    parser.add_argument("--prefix", default="nut", help="file name prefix")
    args, ros_args = parser.parse_known_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = rclpy.create_node("capture_images")
    bridge = CvBridge()
    state = {"frame": None, "pp": None}

    def on_image(msg):
        state["frame"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_info(msg):
        state["pp"] = (int(round(msg.k[2])), int(round(msg.k[5])))

    node.create_subscription(Image, "image_raw", on_image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "camera_info", on_info, qos_profile_sensor_data)
    print(f"Saving to {out_dir.resolve()}. SPACE = save, q = quit.")

    saved = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            frame = state["frame"]
            if frame is not None:
                preview = frame.copy()
                if state["pp"] is not None:
                    cv2.drawMarker(preview, state["pp"], CROSS_COLOUR, cv2.MARKER_CROSS,
                                   CROSS_SIZE, 2)
                cv2.imshow(WINDOW, cv2.resize(preview, None, fx=PREVIEW_SCALE, fy=PREVIEW_SCALE))
            key = cv2.waitKey(KEY_WAIT_MS) & 0xFF
            if key in KEYS_QUIT:
                break
            if key == KEY_SAVE and frame is not None:
                stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")[:-3]
                path = out_dir / f"{args.prefix}_{stamp}.png"
                cv2.imwrite(str(path), frame)
                saved += 1
                print(f"[{saved}] {path}")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
