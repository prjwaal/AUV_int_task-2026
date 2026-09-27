"""
gazebo_adapter.py

The "swap the state source" step the auv_vehicle README named as the
integration point for the Simulation Environment item, done exactly as
described there: vehicle_node.py does not change its telemetry timer,
command handlers, or mission-state-machine wiring at all. It only chooses,
at construction time, which object satisfies the `.step(dt)` /
`.set_target_depth()` / `.set_armed()` / `.state` interface --
VehicleSimModel (pure-Python plant model) or GazeboVehicleAdapter (this
file, backed by a running Gazebo simulation).

What this class does
---------------------
- Publishes a body-frame geometry_msgs/Twist on /model/auv/cmd_vel (bridged
  to Gazebo's VelocityControl system by auv_simulation's ros_gz_bridge
  config) computed by *the same control law* VehicleSimModel uses: a
  first-order depth response toward the commanded target depth, and a
  constant-rate yaw/forward "search" pattern while armed. Reusing the same
  constants (imported from vehicle_sim_model, not re-typed here) means the
  demo behaves the same whether run with use_gazebo:=false or true -- the
  only thing that changes is who is integrating the motion: this class's
  arithmetic, or Gazebo's physics engine.
- Subscribes to /model/auv/odometry (nav_msgs/Odometry, bridged from
  Gazebo's OdometryPublisher system) for ground-truth pose/twist, and
  reshapes the latest sample into the same VehicleSimState dataclass
  VehicleSimModel returns, so vehicle_node's message-construction code
  (which only ever reads `.x`, `.depth`, `.battery_percentage`, etc. off
  whatever `.step()` returns) is unaware which backend produced it.
- Keeps battery drain as a software model, identical to VehicleSimModel's,
  because Gazebo has no battery physics wired into this world. This is a
  deliberate scope cut, not an oversight -- worth naming in the writeup
  alongside the FAULT/auto-return simplification already documented in
  auv_vehicle/README.md.

What it deliberately does NOT do
---------------------------------
Integrate position/orientation itself. That is Gazebo's job now; this
class only ever *reads* pose/twist off /model/auv/odometry and *writes*
a velocity command to /model/auv/cmd_vel. If Gazebo/the bridge is not
running, no odometry ever arrives and the adapter simply keeps publishing
commands into the void, logging a rate-limited warning -- it does not fall
back to VehicleSimModel's kinematics, since silently mixing the two would
make demo behaviour depend on timing/startup order.
"""

import math
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from auv_vehicle.vehicle_sim_model import VehicleSimModel, VehicleSimState

CMD_VEL_TOPIC = "/model/auv/cmd_vel"
ODOMETRY_TOPIC = "/model/auv/odometry"


def _quaternion_to_rpy(q) -> tuple[float, float, float]:
    """geometry_msgs/Quaternion -> (roll, pitch, yaw) in radians."""
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GazeboVehicleAdapter:
    """Drop-in replacement for VehicleSimModel, backed by Gazebo.

    Reuses VehicleSimModel's tuning constants (depth time constant, dive
    rate clamp, search-pattern rate/speed, battery drain rates, fault
    threshold) rather than duplicating them, so the two backends are tuned
    identically by construction.
    """

    BATTERY_FAULT_THRESHOLD_PCT = VehicleSimModel.BATTERY_FAULT_THRESHOLD_PCT

    def __init__(self, node, seed: int | None = None):
        self._node = node
        self._log = node.get_logger()
        self._rng = random.Random(seed)

        self.state = VehicleSimState()
        self._target_depth = 0.0
        self._have_odom = False
        self._warned_no_odom = False

        # Battery is not modelled by Gazebo in this world -- reuse
        # VehicleSimModel purely as a battery/armed-state calculator so the
        # drain formula lives in exactly one place.
        self._battery_model = VehicleSimModel(seed=seed)

        self._cmd_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        node.create_subscription(
            Odometry, ODOMETRY_TOPIC, self._on_odometry, 10
        )

        self._log.info(
            f"GazeboVehicleAdapter active: publishing {CMD_VEL_TOPIC}, "
            f"subscribed to {ODOMETRY_TOPIC}"
        )

    # ------------------------------------------------------------------ #
    # Same public surface as VehicleSimModel
    # ------------------------------------------------------------------ #

    def set_target_depth(self, depth: float) -> None:
        self._target_depth = max(0.0, depth)

    def set_armed(self, armed: bool) -> None:
        self.state.armed = armed
        self._battery_model.set_armed(armed)

    def step(self, dt: float) -> VehicleSimState:
        if not self._have_odom and not self._warned_no_odom:
            self._log.warn(
                f"GazeboVehicleAdapter: no odometry received yet on "
                f"{ODOMETRY_TOPIC} -- is `ros2 launch auv_simulation "
                f"simulation.launch.py` running?",
                throttle_duration_sec=5.0,
            )
            self._warned_no_odom = True

        self._publish_cmd_vel()

        if dt > 0.0:
            self._battery_model.state.armed = self.state.armed
            self._battery_model.step(dt)
            self.state.battery_percentage = self._battery_model.state.battery_percentage
            self.state.battery_voltage = self._battery_model.state.battery_voltage

        return self._with_sensor_noise(self.state)

    # ------------------------------------------------------------------ #
    # Gazebo I/O
    # ------------------------------------------------------------------ #

    def _publish_cmd_vel(self) -> None:
        """Same control law as VehicleSimModel.step(): first-order depth
        response + constant-rate search pattern while armed, expressed as a
        body-frame Twist for Gazebo's VelocityControl system."""
        msg = Twist()

        if not self.state.armed:
            self._cmd_pub.publish(msg)  # all-zero -> hold position
            return

        error = self._target_depth - self.state.depth
        rate = error / VehicleSimModel.DEPTH_TIME_CONSTANT_S
        rate = max(
            -VehicleSimModel.MAX_DIVE_RATE,
            min(VehicleSimModel.MAX_DIVE_RATE, rate),
        )

        msg.linear.x = VehicleSimModel.FORWARD_SPEED_SEARCH
        msg.linear.z = -rate  # world/body +z is up; positive depth is down
        msg.angular.z = VehicleSimModel.YAW_RATE_SEARCH

        self._cmd_pub.publish(msg)

    def _on_odometry(self, msg: Odometry) -> None:
        self._have_odom = True
        p = msg.pose.pose.position
        roll, pitch, yaw = _quaternion_to_rpy(msg.pose.pose.orientation)
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        s = self.state
        s.x, s.y = p.x, p.y
        s.depth = max(0.0, -p.z)
        s.roll, s.pitch, s.yaw = roll, pitch, yaw
        s.vx, s.vy, s.vz = lin.x, lin.y, lin.z
        s.wx, s.wy, s.wz = ang.x, ang.y, ang.z

    def _with_sensor_noise(self, s: VehicleSimState) -> VehicleSimState:
        """Mirrors VehicleSimModel's ground-truth-vs-observation split: the
        Gazebo-reported pose is treated as ground truth, and a lightly
        perturbed copy is what telemetry actually reports -- for the same
        reason VehicleSimModel does this (later estimation/control code
        should chase real sensor noise, not silently work with clean
        simulator ground truth)."""
        noisy = VehicleSimState(**s.__dict__)
        noisy.depth = max(0.0, noisy.depth + self._rng.gauss(0, 0.02))
        noisy.x += self._rng.gauss(0, 0.03)
        noisy.y += self._rng.gauss(0, 0.03)
        noisy.roll += self._rng.gauss(0, 0.01)
        noisy.pitch += self._rng.gauss(0, 0.01)
        return noisy
