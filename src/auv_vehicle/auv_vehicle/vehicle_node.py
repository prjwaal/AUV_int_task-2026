"""
vehicle_node.py

ROS 2 node implementing both the "Vehicle State & Telemetry" component
(Software Head, item 1) and the "Vehicle Command Interface" (item 2).

This supersedes the earlier standalone telemetry_publisher node: commands
must be able to change the same state that telemetry reports (arming the
vehicle has to actually make /auv/telemetry show it armed), so both
responsibilities now share one VehicleSimModel instance inside one node.
telemetry_publisher.py is left in place for isolated component testing, but
vehicle_node is the one to launch for anything involving commands.

Publishes
---------
/auv/telemetry  (auv_interfaces/VehicleState)  -- high rate, BEST_EFFORT
/auv/status     (auv_interfaces/SystemStatus)  -- low rate, RELIABLE + latched
/auv/heartbeat  (auv_interfaces/Heartbeat)      -- fixed rate, RELIABLE

Services
--------
/auv/arm              (auv_interfaces/srv/SetArmed)
/auv/mission_command   (auv_interfaces/srv/MissionCommand)

Action
------
/auv/set_target_depth  (auv_interfaces/action/SetTargetDepth)

Parameters
----------
telemetry_rate_hz     (double, default 20.0)
status_rate_hz        (double, default 2.0)
heartbeat_rate_hz     (double, default 1.0)
sim_seed              (int, default -1)       -1 = unseeded
initial_armed         (bool, default false)
frame_id              (string, default "auv_base_link")
depth_tolerance_m     (double, default 0.15)   default action-goal tolerance
depth_action_timeout_s (double, default 60.0)  abort a stuck dive after this long
depth_feedback_period_s (double, default 0.5)  action feedback publish rate

Design notes
------------
Why services for Arm/Disarm and Start/Stop/Return/Abort, and an action for
Set Target Depth: the former are instantaneous requests with a yes/no/why
answer (a topic gives the caller no acknowledgement at all, and there is no
"progress" to report, so an action would be pure overhead). Reaching a
target depth, by contrast, takes real, variable time and benefits from
feedback and mid-flight cancellation -- exactly what an action is for and a
service cannot do without either blocking the caller or faking the
response before the vehicle has actually arrived.

All mission-state transition logic lives in MissionStateMachine
(mission_state_machine.py, no rclpy dependency, unit-tested in isolation).
This node's job is only to validate ROS-level preconditions, call into the
state machine, apply the resulting side effects to VehicleSimModel (e.g.
commanding depth 0 on RETURN/ABORT to surface), and publish/respond.
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from auv_interfaces.action import SetTargetDepth
from auv_interfaces.msg import Heartbeat, SystemStatus, VehicleState
from auv_interfaces.srv import MissionCommand, SetArmed
from auv_vehicle.mission_state_machine import (
    MissionCommandCode,
    MissionState,
    MissionStateMachine,
)
from auv_vehicle.qos_profiles import HEARTBEAT_QOS, STATUS_QOS, TELEMETRY_QOS
from auv_vehicle.vehicle_sim_model import VehicleSimModel


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (radians) to a geometry_msgs/Quaternion."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


STATUS_MESSAGES = {
    MissionState.DISARMED: "Disarmed",
    MissionState.ARMED: "Armed, standing by",
    MissionState.MISSION_ACTIVE: "Mission active",
    MissionState.MISSION_PAUSED: "Mission paused",
    MissionState.RETURNING: "Returning to home",
    MissionState.FAULT: "Battery critical",
}


class VehicleNode(Node):
    def __init__(self):
        super().__init__("vehicle_node")

        self.declare_parameter("telemetry_rate_hz", 20.0)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("heartbeat_rate_hz", 1.0)
        self.declare_parameter("sim_seed", -1)
        self.declare_parameter("initial_armed", False)
        self.declare_parameter("frame_id", "auv_base_link")
        self.declare_parameter("depth_tolerance_m", 0.15)
        self.declare_parameter("depth_action_timeout_s", 60.0)
        self.declare_parameter("depth_feedback_period_s", 0.5)

        seed = int(self.get_parameter("sim_seed").value)
        self._model = VehicleSimModel(seed=None if seed < 0 else seed)
        self._fsm = MissionStateMachine()

        initial_armed = bool(self.get_parameter("initial_armed").value)
        self._model.set_armed(initial_armed)
        self._fsm.set_armed(initial_armed)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._default_tolerance = float(self.get_parameter("depth_tolerance_m").value)
        self._depth_timeout = float(self.get_parameter("depth_action_timeout_s").value)
        self._feedback_period = float(self.get_parameter("depth_feedback_period_s").value)

        self._heartbeat_seq = 0
        self._last_status_code = None

        # --- Telemetry publishers (unchanged from telemetry_publisher.py) ---
        self._telemetry_pub = self.create_publisher(
            VehicleState, "/auv/telemetry", TELEMETRY_QOS
        )
        self._status_pub = self.create_publisher(
            SystemStatus, "/auv/status", STATUS_QOS
        )
        self._heartbeat_pub = self.create_publisher(
            Heartbeat, "/auv/heartbeat", HEARTBEAT_QOS
        )

        tele_hz = float(self.get_parameter("telemetry_rate_hz").value)
        status_hz = float(self.get_parameter("status_rate_hz").value)
        hb_hz = float(self.get_parameter("heartbeat_rate_hz").value)

        self._last_tick = time.monotonic()
        cb_group = ReentrantCallbackGroup()

        self.create_timer(1.0 / tele_hz, self._on_telemetry_timer, callback_group=cb_group)
        self.create_timer(1.0 / status_hz, self._on_status_timer, callback_group=cb_group)
        self.create_timer(1.0 / hb_hz, self._on_heartbeat_timer, callback_group=cb_group)

        # --- Command Interface: services ---
        self.create_service(
            SetArmed, "/auv/arm", self._on_set_armed, callback_group=cb_group
        )
        self.create_service(
            MissionCommand,
            "/auv/mission_command",
            self._on_mission_command,
            callback_group=cb_group,
        )

        # --- Command Interface: action ---
        self._depth_action_server = ActionServer(
            self,
            SetTargetDepth,
            "/auv/set_target_depth",
            execute_callback=self._execute_set_target_depth,
            goal_callback=self._on_depth_goal,
            cancel_callback=self._on_depth_cancel,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"vehicle_node started (telemetry={tele_hz:.1f}Hz, "
            f"status={status_hz:.1f}Hz, heartbeat={hb_hz:.1f}Hz, "
            f"armed={initial_armed}, state={self._fsm.state.name})"
        )

    # ------------------------------------------------------------------ #
    # Telemetry (same behaviour as telemetry_publisher.py)
    # ------------------------------------------------------------------ #

    def _header(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self._frame_id
        return h

    def _on_telemetry_timer(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        s = self._model.step(dt)

        msg = VehicleState()
        msg.header = self._header()
        msg.position.x = s.x
        msg.position.y = s.y
        msg.position.z = -s.depth
        msg.depth = s.depth
        msg.orientation = rpy_to_quaternion(s.roll, s.pitch, s.yaw)
        msg.linear_velocity.x = s.vx
        msg.linear_velocity.y = s.vy
        msg.linear_velocity.z = s.vz
        msg.angular_velocity.x = s.wx
        msg.angular_velocity.y = s.wy
        msg.angular_velocity.z = s.wz
        msg.battery_voltage = s.battery_voltage
        msg.battery_percentage = s.battery_percentage
        msg.status = int(self._fsm.state)

        self._telemetry_pub.publish(msg)

    def _on_status_timer(self):
        s = self._model.state
        battery_fault = s.battery_percentage < self._model.BATTERY_FAULT_THRESHOLD_PCT
        self._fsm.set_fault(battery_fault)

        msg = SystemStatus()
        msg.header = self._header()
        msg.battery_percentage = s.battery_percentage
        msg.status = int(self._fsm.state)
        msg.status_message = STATUS_MESSAGES[self._fsm.state]

        if msg.status != self._last_status_code:
            self.get_logger().info(
                f"status change -> {msg.status} ({msg.status_message})"
            )
        self._last_status_code = msg.status

        self._status_pub.publish(msg)

    def _on_heartbeat_timer(self):
        msg = Heartbeat()
        msg.header = self._header()
        msg.sequence = self._heartbeat_seq
        self._heartbeat_seq += 1
        self._heartbeat_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Command Interface: Arm/Disarm
    # ------------------------------------------------------------------ #

    def _on_set_armed(self, request, response):
        if request.arm:
            if self._fsm.state == MissionState.FAULT:
                response.success = False
                response.message = "Cannot arm: vehicle is in FAULT state"
            else:
                self._model.set_armed(True)
                self._fsm.set_armed(True)
                response.success = True
                response.message = "Armed"
        else:
            self._model.set_armed(False)
            self._fsm.set_armed(False)
            response.success = True
            response.message = "Disarmed"

        response.resulting_status = int(self._fsm.state)
        self.get_logger().info(
            f"/auv/arm(arm={request.arm}) -> success={response.success} "
            f"({response.message})"
        )
        return response

    # ------------------------------------------------------------------ #
    # Command Interface: Start/Stop/Return/Abort
    # ------------------------------------------------------------------ #

    def _on_mission_command(self, request, response):
        code = MissionCommandCode(request.command)
        result = self._fsm.handle_command(code)

        response.success = result.success
        response.message = result.message
        response.resulting_status = int(result.resulting_state)

        if result.success and code in (
            MissionCommandCode.RETURN,
            MissionCommandCode.ABORT,
        ):
            self._model.set_target_depth(0.0)  # command a surface transit

        self.get_logger().info(
            f"/auv/mission_command({code.name}) -> success={result.success} "
            f"({result.message})"
        )
        return response

    # ------------------------------------------------------------------ #
    # Command Interface: Set Target Depth (action)
    # ------------------------------------------------------------------ #

    def _on_depth_goal(self, goal_request):
        if goal_request.target_depth < 0.0:
            self.get_logger().warn("Rejecting SetTargetDepth: negative depth")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_depth_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_set_target_depth(self, goal_handle):
        goal = goal_handle.request
        target = max(0.0, goal.target_depth)
        tolerance = goal.tolerance if goal.tolerance > 0.0 else self._default_tolerance

        result = SetTargetDepth.Result()

        if self._fsm.state == MissionState.DISARMED:
            goal_handle.abort()
            result.success = False
            result.final_depth = self._model.state.depth
            result.message = "Rejected: vehicle is disarmed"
            return result

        self._model.set_target_depth(target)
        self.get_logger().info(
            f"set_target_depth: target={target:.2f}m tolerance={tolerance:.2f}m"
        )

        feedback = SetTargetDepth.Feedback()
        start_time = time.monotonic()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.final_depth = self._model.state.depth
                result.message = "Cancelled by client"
                return result

            current_depth = self._model.state.depth
            distance = abs(target - current_depth)

            feedback.current_depth = current_depth
            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)

            if distance <= tolerance:
                goal_handle.succeed()
                result.success = True
                result.final_depth = current_depth
                result.message = "Target depth reached"
                return result

            if time.monotonic() - start_time > self._depth_timeout:
                goal_handle.abort()
                result.success = False
                result.final_depth = current_depth
                result.message = "Timed out before reaching target depth"
                return result

            time.sleep(self._feedback_period)

        # rclpy shutting down mid-goal
        if goal_handle.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
            goal_handle.abort()
        result.success = False
        result.final_depth = self._model.state.depth
        result.message = "Node shutting down"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = VehicleNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
