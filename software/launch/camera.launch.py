"""Start the USB camera with its calibration and (optionally) an image viewer.

Usage: ros2 launch software/launch/camera.launch.py [view:=false]
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "software" / "config" / "camera.yaml"
CALIBRATION = REPO / "calibration" / "hd_camera.yaml"


def generate_launch_description():
    view = LaunchConfiguration("view")
    return LaunchDescription([
        DeclareLaunchArgument("view", default_value="true"),
        Node(package="usb_cam", executable="usb_cam_node_exe", name="camera",
             parameters=[str(CONFIG), {"camera_info_url": f"file://{CALIBRATION}"}],
             output="screen"),
        Node(package="rqt_image_view", executable="rqt_image_view",
             arguments=["/image_raw"], condition=IfCondition(view)),
    ])
