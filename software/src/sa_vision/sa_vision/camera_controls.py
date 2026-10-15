"""Apply fixed v4l2 camera controls (exposure, white balance) from camera.yaml.

  ros2 run sa_vision apply_camera_controls [--config software/config/camera.yaml]

usb_cam 0.8 sets exposure and white balance with the control names of kernels
before 5.18 (exposure_auto, white_balance_temperature_auto). Newer kernels reject
them, so its own parameters have no effect and we call v4l2-ctl ourselves.
camera.launch.py runs this before it starts the camera.
"""
import argparse
from pathlib import Path
import subprocess
import sys
from typing import Dict, List, Tuple

import yaml

DEFAULT_CONFIG = Path("software/config/camera.yaml")  # relative to the repository root
V4L2_CTL = "v4l2-ctl"


def load_controls(path: Path) -> Tuple[str, Dict[str, int]]:
    """Video device and the ordered v4l2_controls of camera.yaml."""
    params = yaml.safe_load(Path(path).read_text())["/**"]["ros__parameters"]
    return params["video_device"], dict(params["v4l2_controls"])


def v4l2_commands(device: str, controls: Dict[str, int]) -> List[List[str]]:
    """One v4l2-ctl call per control, in the given order.

    Separate calls keep the order: the kernel rejects a manual value (e.g.
    exposure_time_absolute) while the matching automatic mode is still on.
    """
    return [[V4L2_CTL, "-d", device, "-c", f"{name}={int(value)}"]
            for name, value in controls.items()]


def readback_command(device: str, controls: Dict[str, int]) -> List[str]:
    return [V4L2_CTL, "-d", device, "-C", ",".join(controls)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args, _ = parser.parse_known_args()

    device, controls = load_controls(args.config)
    try:
        for command in v4l2_commands(device, controls):
            subprocess.run(command, check=True)
        readback = subprocess.run(readback_command(device, controls), check=True,
                                  capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"Could not set the camera controls of {device}: {error}", file=sys.stderr)
        sys.exit(1)
    print(f"Camera controls of {device}:\n{readback.stdout.rstrip()}")


if __name__ == "__main__":
    main()
