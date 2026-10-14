"""Camera + nut detection + debug image viewer.

Usage (workspace built and sourced):
  ros2 launch software/launch/vision.launch.py [view:=false] [params_file:=<yaml>]
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_LAUNCH = Path(__file__).resolve().parent / "camera.launch.py"
VISION_PARAMS = Path(get_package_share_directory("sa_vision")) / "config" / "vision.yaml"


def generate_launch_description():
    view = LaunchConfiguration("view")
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("view", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=str(VISION_PARAMS)),
        # Scoped group: view:=false for the camera must not override our own "view".
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(CAMERA_LAUNCH)),
                                              launch_arguments={"view": "false"}.items())]),
        Node(package="sa_vision", executable="vision_node", name="vision_node",
             parameters=[params_file], output="screen"),
        Node(package="rqt_image_view", executable="rqt_image_view",
             arguments=["/vision/debug"], condition=IfCondition(view)),
    ])
