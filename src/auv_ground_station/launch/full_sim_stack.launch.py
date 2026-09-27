"""One-command bringup of the complete item-4 demo: Gazebo + bridge
(auv_simulation) + vehicle_node running with use_gazebo:=true + the ground
station -- everything the Simulation Environment item's "demonstrate
communication between the simulated vehicle, ROS 2 nodes, and the ground
station" line asks for, in a single launch.

This is full_stack.launch.py's sibling, not a replacement for it:
full_stack.launch.py (VehicleSimModel) stays the fast, dependency-light way
to demo items 1-3; this file is what to run for item 4 and for the "with
Gazebo" portion of the demo video.

Usage:
    ros2 launch auv_ground_station full_sim_stack.launch.py
    ros2 launch auv_ground_station full_sim_stack.launch.py initial_armed:=true headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_launch_path = os.path.join(
        get_package_share_directory("auv_simulation"), "launch", "simulation.launch.py"
    )
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
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its GUI window.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch_path),
                launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vehicle_launch_path),
                launch_arguments={
                    "telemetry_rate_hz": LaunchConfiguration("telemetry_rate_hz"),
                    "status_rate_hz": LaunchConfiguration("status_rate_hz"),
                    "heartbeat_rate_hz": LaunchConfiguration("heartbeat_rate_hz"),
                    "initial_armed": LaunchConfiguration("initial_armed"),
                    "sim_seed": LaunchConfiguration("sim_seed"),
                    "use_gazebo": "true",
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
