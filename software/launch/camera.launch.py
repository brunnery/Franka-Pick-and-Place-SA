"""Start the USB camera with fixed exposure, its calibration and (optionally) an image viewer.

Usage (workspace built and sourced): ros2 launch software/launch/camera.launch.py [view:=false]
The v4l2 controls of camera.yaml are applied first; if that fails, nothing is started.
"""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "software" / "config" / "camera.yaml"
CALIBRATION = REPO / "calibration" / "hd_camera.yaml"


def video_device() -> str:
    """Real device behind video_device of camera.yaml.

    camera.yaml names the stable /dev/v4l/by-id/ link (/dev/videoN changes when the
    camera is replugged); usb_cam resolves such relative links wrongly, so it gets
    the target.
    """
    params = yaml.safe_load(CONFIG.read_text())["/**"]["ros__parameters"]
    return os.path.realpath(params["video_device"])


def generate_launch_description():
    view = LaunchConfiguration("view")
    apply_controls = ExecuteProcess(
        cmd=["ros2", "run", "sa_vision", "apply_camera_controls", "--config", str(CONFIG)],
        output="screen")
    camera = Node(package="usb_cam", executable="usb_cam_node_exe", name="camera",
                  parameters=[str(CONFIG), {"camera_info_url": f"file://{CALIBRATION}",
                                                "video_device": video_device()}],
                  output="screen")

    def start_camera(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg="Camera controls not applied, camera not started"),
                    Shutdown(reason="apply_camera_controls failed")]
        return [camera]

    return LaunchDescription([
        DeclareLaunchArgument("view", default_value="true"),
        apply_controls,
        RegisterEventHandler(OnProcessExit(target_action=apply_controls, on_exit=start_camera)),
        Node(package="rqt_image_view", executable="rqt_image_view",
             arguments=["/image_raw"], condition=IfCondition(view)),
    ])
