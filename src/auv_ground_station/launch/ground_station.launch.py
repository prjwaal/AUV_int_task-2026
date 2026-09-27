"""Launch the ground_station GUI on its own (vehicle must already be running).

Usage:
    ros2 launch auv_ground_station ground_station.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_heartbeat_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("heartbeat_timeout_multiplier", default_value="3.0"),
            Node(
                package="auv_ground_station",
                executable="ground_station",
                name="ground_station",
                output="screen",
                parameters=[
                    {
                        "vehicle_heartbeat_rate_hz": LaunchConfiguration(
                            "vehicle_heartbeat_rate_hz"
                        ),
                        "heartbeat_timeout_multiplier": LaunchConfiguration(
                            "heartbeat_timeout_multiplier"
                        ),
                    }
                ],
            ),
        ]
    )
