"""Quick launch for inspecting available TF frames from libsurvive_ros2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "duration",
            default_value="5.0",
            description="How long to listen for TF messages (seconds).",
        ),
        Node(
            package="primeu_teleop",
            executable="tf_inspector",
            name="tf_inspector",
            output="screen",
            emulate_tty=True,
            parameters=[{"duration": LaunchConfiguration("duration")}],
        ),
    ])
