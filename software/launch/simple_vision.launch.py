"""Camera + simple_vision (one object on a blank background) + debug image viewer.

Usage (workspace built and sourced):
  ros2 launch software/launch/simple_vision.launch.py [view:=false]
Same node name and topics as vision.launch.py (sa_vision), so sa_tasks works with
either; start only one of the two.
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
PARAMS = Path(get_package_share_directory("simple_vision")) / "config" / "simple_vision.yaml"


def generate_launch_description():
    view = LaunchConfiguration("view")
    return LaunchDescription([
        DeclareLaunchArgument("view", default_value="true"),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(CAMERA_LAUNCH)),
                                              launch_arguments={"view": "false"}.items())]),
        Node(package="simple_vision", executable="simple_vision", name="vision_node",
             parameters=[str(PARAMS)], output="screen"),
        Node(package="rqt_image_view", executable="rqt_image_view",
             arguments=["/vision/debug"], condition=IfCondition(view)),
    ])
