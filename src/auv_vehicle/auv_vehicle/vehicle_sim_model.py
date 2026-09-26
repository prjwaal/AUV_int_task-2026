"""
Minimal AUV plant model used to generate plausible telemetry in the absence
of a connected physics simulator (Gazebo/Webots).

Deliberately kept separate from the ROS 2 node (telemetry_publisher.py) so
that:
  1. It has zero dependency on rclpy and can be unit-tested in isolation
     (see test/test_vehicle_sim_model.py) without a ROS 2 install.
  2. It can later be swapped for a thin adapter that consumes ground-truth
     state from Gazebo/Webots (item 4, Simulation Environment) without any
     change to the publishing node -- the node only ever talks to whatever
     object satisfies this same `.step(dt) -> VehicleSimState` interface.

The model is intentionally simple -- a first-order depth response toward a
commanded target depth, a constant-rate heading "search" pattern while
armed, and linear battery drain proportional to elapsed time and whether the
vehicle is armed. It is NOT hydrodynamically accurate: it exists to produce
smooth, physically-reasonable telemetry for the Ground Station demo, and as
a stand-in the Command Interface (item 2) and Mission Management can drive
via set_target_depth()/set_armed() once they exist.
"""

import math
import random
from dataclasses import dataclass


@dataclass
class VehicleSimState:
    x: float = 0.0
    y: float = 0.0
    depth: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    battery_voltage: float = 16.8   # 4S Li-ion pack, fully charged
    battery_percentage: float = 100.0
    armed: bool = False


class VehicleSimModel:
    """First-order plant model with light sensor-noise injection."""

    BATTERY_MIN_V = 13.2            # pack cutoff voltage
    BATTERY_MAX_V = 16.8
    DEPTH_TIME_CONSTANT_S = 4.0      # 1st-order lag toward commanded depth
    MAX_DIVE_RATE = 0.6              # m/s clamp on vertical speed
    YAW_RATE_SEARCH = 0.15           # rad/s while idly patrolling armed
    FORWARD_SPEED_SEARCH = 0.3       # m/s while idly patrolling armed
    DRAIN_PCT_PER_HOUR_ARMED = 6.0
    DRAIN_PCT_PER_HOUR_IDLE = 0.4
    BATTERY_FAULT_THRESHOLD_PCT = 15.0

    def __init__(self, seed: int | None = None):
        self.state = VehicleSimState()
        self._target_depth = 0.0
        self._rng = random.Random(seed)

    def set_target_depth(self, depth: float) -> None:
        """Set the commanded depth (metres, clamped to >= 0)."""
        self._target_depth = max(0.0, depth)

    def set_armed(self, armed: bool) -> None:
        self.state.armed = armed

    def step(self, dt: float) -> VehicleSimState:
        """Advance the model by dt seconds and return a *noisy* observed
        state (the internal `self.state` stays noise-free so control loops
        built on top of this model later are not chasing their own sensor
        noise)."""
        if dt <= 0.0:
            return self.state

        s = self.state

        # --- Depth: first-order response toward target, rate-limited ---
        error = self._target_depth - s.depth
        rate = error / self.DEPTH_TIME_CONSTANT_S
        rate = max(-self.MAX_DIVE_RATE, min(self.MAX_DIVE_RATE, rate))
        s.vz = rate if s.armed else 0.0
        s.depth = max(0.0, s.depth + s.vz * dt)

        # --- Heading / horizontal motion: search pattern while armed ---
        if s.armed:
            s.wz = self.YAW_RATE_SEARCH
            s.yaw = (s.yaw + s.wz * dt) % (2 * math.pi)
            s.vx = self.FORWARD_SPEED_SEARCH
            s.x += s.vx * math.cos(s.yaw) * dt
            s.y += s.vx * math.sin(s.yaw) * dt
        else:
            s.wz = 0.0
            s.vx = 0.0

        # --- Battery drain ---
        drain_per_hour = (
            self.DRAIN_PCT_PER_HOUR_ARMED if s.armed else self.DRAIN_PCT_PER_HOUR_IDLE
        )
        s.battery_percentage = max(
            0.0, s.battery_percentage - drain_per_hour * dt / 3600.0
        )
        span = self.BATTERY_MAX_V - self.BATTERY_MIN_V
        s.battery_voltage = self.BATTERY_MIN_V + span * (s.battery_percentage / 100.0)

        return self._with_sensor_noise(s)

    def _with_sensor_noise(self, s: VehicleSimState) -> VehicleSimState:
        """Return a copy of `s` with small Gaussian noise applied to the
        fields a real sensor suite would not measure perfectly. The
        underlying `self.state` (ground truth) is left untouched."""
        noisy = VehicleSimState(**s.__dict__)
        noisy.depth = max(0.0, noisy.depth + self._rng.gauss(0, 0.02))
        noisy.x += self._rng.gauss(0, 0.03)
        noisy.y += self._rng.gauss(0, 0.03)
        noisy.roll += self._rng.gauss(0, 0.01)
        noisy.pitch += self._rng.gauss(0, 0.01)
        return noisy
