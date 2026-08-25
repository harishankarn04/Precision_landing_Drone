"""Start the MAVLink bridge and the stand-in target publisher together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connection = LaunchConfiguration("connection")
    pad_north = LaunchConfiguration("pad_north")
    pad_east = LaunchConfiguration("pad_east")

    return LaunchDescription([
        DeclareLaunchArgument(
            "connection",
            default_value="tcp:host.docker.internal:5762",
            description="MAVLink endpoint for the autopilot (SITL SERIAL1).",
        ),
        DeclareLaunchArgument("pad_north", default_value="0.0"),
        DeclareLaunchArgument("pad_east", default_value="0.0"),

        Node(
            package="precision_landing",
            executable="mavlink_bridge",
            name="mavlink_bridge",
            output="screen",
            parameters=[{"connection": connection}],
        ),
        Node(
            package="precision_landing",
            executable="fake_target",
            name="fake_target",
            output="screen",
            parameters=[{"pad_north": pad_north, "pad_east": pad_east}],
        ),
    ])
