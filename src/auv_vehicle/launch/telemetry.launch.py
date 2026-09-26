"""Launch vehicle_node: telemetry publishing plus the command interface
(Arm/Disarm, Start/Stop/Return/Abort, Set Target Depth).

This supersedes launching telemetry_publisher directly -- vehicle_node is a
superset (same telemetry topics, plus the /auv/arm, /auv/mission_command
services and the /auv/set_target_depth action) so the same launch command
you already use keeps working, and now arm/mission/depth commands actually
affect what telemetry reports.

Usage:
    ros2 launch auv_vehicle telemetry.launch.py
    ros2 launch auv_vehicle telemetry.launch.py initial_armed:=true telemetry_rate_hz:=50.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("telemetry_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("status_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("heartbeat_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("initial_armed", default_value="false"),
            DeclareLaunchArgument("sim_seed", default_value="-1"),
            DeclareLaunchArgument("depth_tolerance_m", default_value="0.15"),
            DeclareLaunchArgument("depth_action_timeout_s", default_value="60.0"),
            DeclareLaunchArgument("depth_feedback_period_s", default_value="0.5"),
            Node(
                package="auv_vehicle",
                executable="vehicle_node",
                name="vehicle_node",
                output="screen",
                parameters=[
                    {
                        "telemetry_rate_hz": LaunchConfiguration("telemetry_rate_hz"),
                        "status_rate_hz": LaunchConfiguration("status_rate_hz"),
                        "heartbeat_rate_hz": LaunchConfiguration("heartbeat_rate_hz"),
                        "initial_armed": LaunchConfiguration("initial_armed"),
                        "sim_seed": LaunchConfiguration("sim_seed"),
                        "depth_tolerance_m": LaunchConfiguration("depth_tolerance_m"),
                        "depth_action_timeout_s": LaunchConfiguration(
                            "depth_action_timeout_s"
                        ),
                        "depth_feedback_period_s": LaunchConfiguration(
                            "depth_feedback_period_s"
                        ),
                    }
                ],
            ),
        ]
    )
