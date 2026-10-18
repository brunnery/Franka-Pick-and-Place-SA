"""FR3 bringup without a motion controller: robot state for teach_poses.

Usage (franka workspace and software/ sourced):
  ros2 launch software/launch/robot_state.launch.py [robot_ip:=192.168.1.200]
Starts franka_bringup/franka.launch.py with the controllers.yaml of
cartesian_impedance_control (joint_state_broadcaster + franka_robot_state_broadcaster),
but spawns no controller, so the arm can be guided by hand with the pilot buttons.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.200"),
        DeclareLaunchArgument("robot_type", default_value="fr3"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare("franka_bringup"), "launch", "franka.launch.py"])]),
            launch_arguments={
                "robot_ip": LaunchConfiguration("robot_ip"),
                "robot_type": LaunchConfiguration("robot_type"),
                "load_gripper": "false",
                "use_fake_hardware": "false",
                "fake_sensor_commands": "false",
                "controllers_yaml": PathJoinSubstitution(
                    [FindPackageShare("cartesian_impedance_control"), "launch",
                     "controllers.yaml"]),
            }.items(),
        ),
    ])
