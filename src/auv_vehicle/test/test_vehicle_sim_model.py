"""
Unit tests for VehicleSimModel. These import nothing from rclpy and can run
with plain pytest -- no ROS 2 environment needs to be sourced.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from auv_vehicle.vehicle_sim_model import VehicleSimModel  # noqa: E402


def test_depth_converges_to_target_when_armed():
    model = VehicleSimModel(seed=42)
    model.set_armed(True)
    model.set_target_depth(10.0)

    state = model.state
    for _ in range(2000):
        state = model.step(0.05)

    assert abs(state.depth - 10.0) < 0.5


def test_depth_holds_when_disarmed():
    model = VehicleSimModel(seed=7)
    model.set_target_depth(20.0)  # commanded, but vehicle is not armed
    model.set_armed(False)

    state = model.state
    for _ in range(200):
        state = model.step(0.1)

    assert state.depth < 1.0  # should not have descended toward target


def test_disarmed_vehicle_does_not_move_horizontally():
    model = VehicleSimModel(seed=1)
    model.set_armed(False)

    state = model.state
    for _ in range(100):
        state = model.step(0.1)

    assert state.vx == 0.0
    assert state.wz == 0.0


def test_battery_drains_faster_when_armed():
    armed_model = VehicleSimModel(seed=2)
    armed_model.set_armed(True)
    idle_model = VehicleSimModel(seed=2)
    idle_model.set_armed(False)

    armed_state = armed_model.state
    idle_state = idle_model.state
    for _ in range(50):
        armed_state = armed_model.step(1.0)
        idle_state = idle_model.step(1.0)

    assert armed_state.battery_percentage < idle_state.battery_percentage


def test_battery_voltage_tracks_percentage_monotonically():
    model = VehicleSimModel(seed=3)
    model.set_armed(True)

    prev_pct = 100.0
    prev_v = model.state.battery_voltage
    for _ in range(20):
        state = model.step(60.0)  # big steps to force noticeable drain
        assert state.battery_percentage <= prev_pct
        assert state.battery_voltage <= prev_v
        prev_pct = state.battery_percentage
        prev_v = state.battery_voltage


def test_step_with_zero_dt_is_a_no_op():
    model = VehicleSimModel(seed=5)
    model.set_armed(True)
    model.set_target_depth(5.0)
    model.step(1.0)
    before = model.state.depth
    result = model.step(0.0)
    assert result.depth == before
