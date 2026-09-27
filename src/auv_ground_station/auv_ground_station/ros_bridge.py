"""
ros_bridge.py

Bridges rclpy (running on its own background thread) to the PySide6 GUI
(running on the main thread) for the Ground Station (Software Head,
item 3).

Threading model
----------------
- `GroundStationNode` (rclpy.Node) and its executor run on one dedicated
  background thread (see app.py) that repeatedly calls
  `executor.spin_once()` and then drains `RosBridge.command_queue` -- a
  plain thread-safe `queue.Queue` -- for outgoing commands requested by
  the GUI. This confines every rclpy API call (subscriptions, service and
  action clients) to a single thread, sidestepping rclpy's general lack of
  guarantees about concurrent calls from multiple threads.
- Incoming data (telemetry/status/heartbeat/command results) reaches the
  GUI via Qt signals emitted from that background thread. This is safe
  without extra locking: PySide6 delivers a signal according to the
  *receiving* QObject's thread affinity, not the emitting thread's, so as
  long as `MainWindow` is constructed on the main thread (it is -- see
  app.py) and the main thread is running the Qt event loop (`app.exec()`),
  these cross-thread emissions are automatically queued and delivered
  safely.
- The reverse direction (GUI -> ROS) deliberately does NOT rely on Qt's
  automatic queued-connection delivery, because that requires the
  *receiving* object's thread to be running a Qt event loop -- and the
  background thread here runs a plain rclpy spin loop, not one. Instead,
  GUI button handlers simply put a command tuple on `command_queue`;
  `GroundStationNode.handle_command()` drains and executes it from within
  the ROS thread's own loop. This is simpler and more robust than trying
  to make Qt's cross-thread signal delivery work into a non-Qt thread.
"""

import time

from PySide6.QtCore import QObject, Signal
from rclpy.action import ActionClient
from rclpy.node import Node

from auv_interfaces.action import SetTargetDepth
from auv_interfaces.msg import Heartbeat, SystemStatus, VehicleState
from auv_interfaces.srv import MissionCommand, SetArmed
from auv_vehicle.qos_profiles import HEARTBEAT_QOS, STATUS_QOS, TELEMETRY_QOS

from auv_ground_station.comms_monitor import CommsMonitor


class RosBridge(QObject):
    """Qt-facing signal hub. Created on the main thread; emitted from the
    background ROS thread. The GUI only ever reads these signals and
    writes to `command_queue` -- it never imports rclpy or touches the
    node directly."""

    telemetry_received = Signal(dict)
    status_received = Signal(dict)
    comms_status_changed = Signal(bool)  # True = OK, False = lost
    command_result = Signal(str, bool, str)  # command name, success, message
    depth_feedback = Signal(float, float)  # current_depth, distance_remaining
    depth_action_result = Signal(bool, float, str)  # success, final_depth, message

    def __init__(self):
        super().__init__()
        import queue

        self.command_queue: "queue.Queue" = queue.Queue()


class GroundStationNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__("ground_station")
        self._bridge = bridge

        self.declare_parameter("vehicle_heartbeat_rate_hz", 1.0)
        self.declare_parameter("heartbeat_timeout_multiplier", 3.0)

        hb_rate = float(self.get_parameter("vehicle_heartbeat_rate_hz").value)
        multiplier = float(self.get_parameter("heartbeat_timeout_multiplier").value)
        self._comms = CommsMonitor(timeout_s=multiplier / hb_rate)
        self._comms_ok_last = None
        self._current_depth_goal_handle = None

        self.create_subscription(
            VehicleState, "/auv/telemetry", self._on_telemetry, TELEMETRY_QOS
        )
        self.create_subscription(
            SystemStatus, "/auv/status", self._on_status, STATUS_QOS
        )
        self.create_subscription(
            Heartbeat, "/auv/heartbeat", self._on_heartbeat, HEARTBEAT_QOS
        )

        self._arm_client = self.create_client(SetArmed, "/auv/arm")
        self._mission_client = self.create_client(MissionCommand, "/auv/mission_command")
        self._depth_action_client = ActionClient(
            self, SetTargetDepth, "/auv/set_target_depth"
        )

        self.create_timer(0.5, self._check_comms)

        self.get_logger().info(
            f"ground_station started (comms timeout={self._comms.timeout_s:.1f}s)"
        )

    # ------------------------------------------------------------------ #
    # Subscriptions -> Qt signals
    # ------------------------------------------------------------------ #

    def _on_telemetry(self, msg: VehicleState):
        self._bridge.telemetry_received.emit(
            {
                "depth": msg.depth,
                "x": msg.position.x,
                "y": msg.position.y,
                "z": msg.position.z,
                "vx": msg.linear_velocity.x,
                "vy": msg.linear_velocity.y,
                "vz": msg.linear_velocity.z,
                "battery_voltage": msg.battery_voltage,
                "battery_percentage": msg.battery_percentage,
                "status": msg.status,
            }
        )

    def _on_status(self, msg: SystemStatus):
        self._bridge.status_received.emit(
            {
                "status": msg.status,
                "battery_percentage": msg.battery_percentage,
                "status_message": msg.status_message,
            }
        )

    def _on_heartbeat(self, msg: Heartbeat):
        self._comms.on_heartbeat(time.monotonic())

    def _check_comms(self):
        ok = not self._comms.is_lost(time.monotonic())
        if ok != self._comms_ok_last:
            self._bridge.comms_status_changed.emit(ok)
            self._comms_ok_last = ok

    # ------------------------------------------------------------------ #
    # Outgoing commands -- invoked only from the ROS thread's own loop,
    # never called directly by the GUI (see command_queue in ros_bridge).
    # ------------------------------------------------------------------ #

    def handle_command(self, command: tuple):
        kind = command[0]
        if kind == "arm":
            self._send_arm(command[1])
        elif kind == "mission":
            self._send_mission_command(command[1])
        elif kind == "set_depth":
            self._send_set_depth(command[1], command[2])
        elif kind == "cancel_depth":
            self._cancel_depth_goal()
        else:
            self.get_logger().warn(f"Unknown command kind: {kind}")

    def _send_arm(self, arm: bool):
        if not self._arm_client.wait_for_service(timeout_sec=1.0):
            self._bridge.command_result.emit("arm", False, "Service unavailable")
            return
        req = SetArmed.Request()
        req.arm = arm
        future = self._arm_client.call_async(req)
        future.add_done_callback(self._on_arm_response)

    def _on_arm_response(self, future):
        try:
            resp = future.result()
            self._bridge.command_result.emit("arm", resp.success, resp.message)
        except Exception as exc:  # noqa: BLE001
            self._bridge.command_result.emit("arm", False, f"Call failed: {exc}")

    def _send_mission_command(self, code: int):
        if not self._mission_client.wait_for_service(timeout_sec=1.0):
            self._bridge.command_result.emit("mission", False, "Service unavailable")
            return
        req = MissionCommand.Request()
        req.command = code
        future = self._mission_client.call_async(req)
        future.add_done_callback(self._on_mission_response)

    def _on_mission_response(self, future):
        try:
            resp = future.result()
            self._bridge.command_result.emit("mission", resp.success, resp.message)
        except Exception as exc:  # noqa: BLE001
            self._bridge.command_result.emit("mission", False, f"Call failed: {exc}")

    def _send_set_depth(self, target_depth: float, tolerance: float):
        if not self._depth_action_client.wait_for_server(timeout_sec=1.0):
            self._bridge.command_result.emit(
                "set_depth", False, "Action server unavailable"
            )
            return
        goal = SetTargetDepth.Goal()
        goal.target_depth = target_depth
        goal.tolerance = tolerance
        send_future = self._depth_action_client.send_goal_async(
            goal, feedback_callback=self._on_depth_feedback
        )
        send_future.add_done_callback(self._on_depth_goal_response)

    def _on_depth_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._bridge.command_result.emit(
                "set_depth", False, "Goal rejected by vehicle"
            )
            return
        self._current_depth_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_depth_result)

    def _on_depth_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._bridge.depth_feedback.emit(fb.current_depth, fb.distance_remaining)

    def _on_depth_result(self, future):
        result = future.result().result
        self._bridge.depth_action_result.emit(
            result.success, result.final_depth, result.message
        )
        self._current_depth_goal_handle = None

    def _cancel_depth_goal(self):
        if self._current_depth_goal_handle is not None:
            self._current_depth_goal_handle.cancel_goal_async()
