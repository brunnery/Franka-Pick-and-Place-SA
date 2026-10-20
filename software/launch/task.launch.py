"""Robot (impedance controller) + vision + one run of a sa_tasks step.

Usage (franka_ros2_ws and workspace built and sourced, PYTHONNOUSERSITE=1):
  ros2 launch software/launch/task.launch.py step:=<pick|place_screw|full>
      [approach_only:=true] [view:=false] [record:=true]

approach_only (place_screw): stud search, then stop and hold above the thread tip.
The step starts once the spawner of cartesian_impedance_controller has exited successfully
(controller active), not after a fixed delay. Robot and vision keep running after the step
ends; Ctrl+C stops everything. Note: the gripper watchdog lets go 2 s after the step's
process ends (docs/PLAN.md Section 6).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VISION_LAUNCH = Path(__file__).resolve().parent / "vision.launch.py"
CONTROLLER_LAUNCH = (Path(get_package_share_directory("cartesian_impedance_control"))
                     / "launch" / "cartesian_impedance_controller.launch.py")
CONTROLLER = "cartesian_impedance_controller"
STEPS = ("pick", "place_screw", "full")


def start_step_when_controller_active(event, context):
    # Every spawner of franka.launch.py ends with a ProcessExited event; react only to ours
    if CONTROLLER not in event.cmd:
        return None
    step = LaunchConfiguration("step").perform(context)
    if event.returncode != 0:
        return [LogInfo(msg=f"spawning {CONTROLLER} failed: {step} not started")]
    if step not in STEPS:
        return [LogInfo(msg=f"step must be one of {STEPS}, not '{step}': nothing started")]
    # locate_only was removed (20.10.): refuse it, else an old alias would run the full step
    if "locate_only" in context.launch_configurations:
        return [LogInfo(msg="locate_only no longer exists, use approach_only:=true: "
                            "nothing started")]
    arguments = []
    if LaunchConfiguration("approach_only").perform(context).lower() in ("true", "1"):
        arguments.append("--approach-only")
    return [LogInfo(msg=f"{CONTROLLER} active: starting {step} {' '.join(arguments)}"),
            Node(package="sa_tasks", executable=step, arguments=arguments, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("step", description="sa_tasks step: " + " | ".join(STEPS)),
        DeclareLaunchArgument("approach_only", default_value="false",
                              description="place_screw: stop and hold above the thread tip."),
        DeclareLaunchArgument("view", default_value="true",
                              description="Show /vision/debug in rqt_image_view."),
        DeclareLaunchArgument("record", default_value="false",
                              description="Record the controller debug topics with ros2 bag."),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(CONTROLLER_LAUNCH)),
            # No start move of the controller: the step goes to its observation pose itself
            launch_arguments={"record": LaunchConfiguration("record"),
                              "start_pose": "false"}.items())]),
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(VISION_LAUNCH)),
            launch_arguments={"view": LaunchConfiguration("view")}.items())]),
        RegisterEventHandler(OnProcessExit(on_exit=start_step_when_controller_active)),
    ])
