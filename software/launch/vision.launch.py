"""Camera + nut detection + debug image viewer.

Usage (workspace built and sourced):
  ros2 launch software/launch/vision.launch.py [view:=false] [params_file:=<yaml>]
      [target:=stud] [distance_mm:=425]   # by hand: target and d instead of vision.yaml
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_LAUNCH = Path(__file__).resolve().parent / "camera.launch.py"
VISION_PARAMS = Path(get_package_share_directory("sa_vision")) / "config" / "vision.yaml"


def vision_node(context):
    """vision.yaml, with target / distance_mm overriding it when given (sa_tasks sets both
    itself with every request)."""
    overrides = {}
    if LaunchConfiguration("target").perform(context):
        overrides["target"] = LaunchConfiguration("target").perform(context)
    if LaunchConfiguration("distance_mm").perform(context):
        overrides["object_plane_distance_mm"] = float(
            LaunchConfiguration("distance_mm").perform(context))
    return [Node(package="sa_vision", executable="vision_node", name="vision_node",
                 parameters=[LaunchConfiguration("params_file").perform(context), overrides],
                 output="screen")]


def generate_launch_description():
    view = LaunchConfiguration("view")
    return LaunchDescription([
        DeclareLaunchArgument("view", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=str(VISION_PARAMS)),
        # Scoped group: view:=false for the camera must not override our own "view".
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(CAMERA_LAUNCH)),
                                              launch_arguments={"view": "false"}.items())]),
        DeclareLaunchArgument("target", default_value="", description="nut | stud"),
        DeclareLaunchArgument("distance_mm", default_value="",
                              description="object_plane_distance_mm"),
        OpaqueFunction(function=vision_node),
        Node(package="rqt_image_view", executable="rqt_image_view",
             arguments=["/vision/debug"], condition=IfCondition(view)),
    ])
