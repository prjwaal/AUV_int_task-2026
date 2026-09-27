"""One-command bringup: vehicle_node + ground_station together.

Convenience wrapper for demoing/recording -- everything the Software Head
components need in a single launch, instead of two separate terminals.

Usage:
    ros2 launch auv_ground_station full_stack.launch.py
    ros2 launch auv_ground_station full_stack.launch.py initial_armed:=true sim_seed:=42
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_launch_path = os.path.join(
        get_package_share_directory("auv_vehicle"), "launch", "telemetry.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("telemetry_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("status_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("heartbeat_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("initial_armed", default_value="false"),
            DeclareLaunchArgument("sim_seed", default_value="-1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vehicle_launch_path),
                launch_arguments={
                    "telemetry_rate_hz": LaunchConfiguration("telemetry_rate_hz"),
                    "status_rate_hz": LaunchConfiguration("status_rate_hz"),
                    "heartbeat_rate_hz": LaunchConfiguration("heartbeat_rate_hz"),
                    "initial_armed": LaunchConfiguration("initial_armed"),
                    "sim_seed": LaunchConfiguration("sim_seed"),
                }.items(),
            ),
            Node(
                package="auv_ground_station",
                executable="ground_station",
                name="ground_station",
                output="screen",
                parameters=[
                    {
                        "vehicle_heartbeat_rate_hz": LaunchConfiguration(
                            "heartbeat_rate_hz"
                        ),
                        "heartbeat_timeout_multiplier": 3.0,
                    }
                ],
            ),
        ]
    )
