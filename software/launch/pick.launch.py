"""Robot + vision + one pick run: task.launch.py with step:=pick (alias sa_pick_all).

  ros2 launch software/launch/pick.launch.py [locate_only:=true] [view:=false] [record:=true]
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

TASK_LAUNCH = Path(__file__).resolve().parent / "task.launch.py"


def generate_launch_description():
    # The other arguments (locate_only, view, record) pass through to the include
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(TASK_LAUNCH)),
        launch_arguments={"step": "pick"}.items())])
