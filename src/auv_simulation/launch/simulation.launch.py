"""Bring up Gazebo (the AUV world + model) and the ros_gz_bridge that
connects it to ROS 2.

This launch file only starts the *simulator side*: Gazebo itself and the
bridge translating /model/auv/cmd_vel and /model/auv/odometry (and /clock)
between Gazebo Transport and ROS 2 topics. It does not start vehicle_node
or the ground station -- see auv_ground_station/launch/full_sim_stack.launch.py
for the one-command bringup of the complete stack.

Usage:
    ros2 launch auv_simulation simulation.launch.py
    ros2 launch auv_simulation simulation.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("auv_simulation")
    world_path = os.path.join(pkg_share, "worlds", "auv_world.sdf")
    models_path = os.path.join(pkg_share, "models")
    bridge_config = os.path.join(pkg_share, "config", "auv_bridge.yaml")

    headless = LaunchConfiguration("headless")
    gz_sim_launch = PathJoinSubstitution(
        [get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run gz sim with -s (server only, no GUI) -- "
                "useful for a CI/headless machine or when only the ROS 2 "
                "side of the demo needs to be shown.",
            ),
            # So "<include><uri>model://auv</uri></include>" in the world
            # file resolves to models/auv/model.sdf.
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=models_path
                + (
                    ":" + os.environ["GZ_SIM_RESOURCE_PATH"]
                    if "GZ_SIM_RESOURCE_PATH" in os.environ
                    else ""
                ),
            ),
            # GUI + server (default)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": f"-r {world_path}"}.items(),
                condition=UnlessCondition(headless),
            ),
            # Server only
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": f"-s -r {world_path}"}.items(),
                condition=IfCondition(headless),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="auv_gz_bridge",
                output="screen",
                parameters=[{"config_file": bridge_config}],
            ),
        ]
    )
