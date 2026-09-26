"""
telemetry_publisher.py

ROS 2 node implementing the "Vehicle State & Telemetry" component
(Software Head, assessment item 1).

Publishes
---------
/auv/telemetry  (auv_interfaces/VehicleState)  -- high rate, BEST_EFFORT
/auv/status     (auv_interfaces/SystemStatus)  -- low rate, RELIABLE + latched
/auv/heartbeat  (auv_interfaces/Heartbeat)      -- fixed rate, RELIABLE

Parameters
----------
telemetry_rate_hz  (double, default 20.0)  publish rate of /auv/telemetry
status_rate_hz     (double, default 2.0)   publish rate of /auv/status
heartbeat_rate_hz  (double, default 1.0)   publish rate of /auv/heartbeat
sim_seed           (int, default -1)       RNG seed for sensor noise; -1 = unseeded
initial_armed      (bool, default false)   starting arm state
frame_id           (string, default "auv_base_link")

Design notes
------------
This node deliberately contains no vehicle dynamics of its own -- all of
that lives in VehicleSimModel (vehicle_sim_model.py), a plain-Python class
with no rclpy dependency. This node's only responsibilities are ROS
wiring: parameter declaration, timers, message construction, and QoS
selection (see qos_profiles.py for the reasoning behind each choice).

The upside of this split is that when the Simulation Environment item
(Gazebo/Webots) is wired in, only the ~5-line `_on_telemetry_timer` state
source needs to change (swap `self._model.step(dt)` for a subscription
callback populated from simulator ground truth); every QoS decision,
parameter, and message-construction line is untouched. Three topics rather
than one were chosen because telemetry, status, and liveness have three
different reliability/durability requirements (see qos_profiles.py) that
cannot all be satisfied by a single QoS profile on a single topic.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from std_msgs.msg import Header

from auv_interfaces.msg import Heartbeat, SystemStatus, VehicleState
from auv_vehicle.qos_profiles import HEARTBEAT_QOS, STATUS_QOS, TELEMETRY_QOS
from auv_vehicle.vehicle_sim_model import VehicleSimModel


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (radians) to a geometry_msgs/Quaternion.
    Implemented locally to avoid pulling in tf_transformations as a
    dependency for a single conversion."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__("telemetry_publisher")

        self.declare_parameter("telemetry_rate_hz", 20.0)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("heartbeat_rate_hz", 1.0)
        self.declare_parameter("sim_seed", -1)
        self.declare_parameter("initial_armed", False)
        self.declare_parameter("frame_id", "auv_base_link")

        seed = int(self.get_parameter("sim_seed").value)
        self._model = VehicleSimModel(seed=None if seed < 0 else seed)
        self._model.set_armed(bool(self.get_parameter("initial_armed").value))
        self._frame_id = str(self.get_parameter("frame_id").value)

        self._heartbeat_seq = 0
        self._last_status_code = None

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
        self.create_timer(1.0 / tele_hz, self._on_telemetry_timer)
        self.create_timer(1.0 / status_hz, self._on_status_timer)
        self.create_timer(1.0 / hb_hz, self._on_heartbeat_timer)

        self.get_logger().info(
            f"telemetry_publisher started "
            f"(telemetry={tele_hz:.1f}Hz, status={status_hz:.1f}Hz, "
            f"heartbeat={hb_hz:.1f}Hz, armed={self._model.state.armed})"
        )

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
        msg.position.z = -s.depth  # ENU convention: down is negative z
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
        msg.status = (
            VehicleState.STATUS_ARMED if s.armed else VehicleState.STATUS_DISARMED
        )

        self._telemetry_pub.publish(msg)

    def _on_status_timer(self):
        s = self._model.state  # ground-truth (non-noisy) state for status logic

        msg = SystemStatus()
        msg.header = self._header()
        msg.battery_percentage = s.battery_percentage

        if s.battery_percentage < self._model.BATTERY_FAULT_THRESHOLD_PCT:
            msg.status = SystemStatus.STATUS_FAULT
            msg.status_message = (
                f"Battery critical ({s.battery_percentage:.1f}% < "
                f"{self._model.BATTERY_FAULT_THRESHOLD_PCT:.0f}%)"
            )
        elif s.armed:
            msg.status = SystemStatus.STATUS_ARMED
            msg.status_message = "Nominal"
        else:
            msg.status = SystemStatus.STATUS_DISARMED
            msg.status_message = "Standby"

        if msg.status != self._last_status_code:
            self.get_logger().info(
                f"status change: {self._last_status_code} -> {msg.status} "
                f"({msg.status_message})"
            )
        self._last_status_code = msg.status

        self._status_pub.publish(msg)

    def _on_heartbeat_timer(self):
        msg = Heartbeat()
        msg.header = self._header()
        msg.sequence = self._heartbeat_seq
        self._heartbeat_seq += 1
        self._heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
