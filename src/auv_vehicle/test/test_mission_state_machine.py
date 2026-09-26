"""
Unit tests for MissionStateMachine. Pure Python, no rclpy import -- runnable
with plain pytest.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from auv_vehicle.mission_state_machine import (  # noqa: E402
    MissionCommandCode,
    MissionState,
    MissionStateMachine,
)


def test_starts_disarmed():
    fsm = MissionStateMachine()
    assert fsm.state == MissionState.DISARMED


def test_arm_from_disarmed_moves_to_armed():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    assert fsm.state == MissionState.ARMED


def test_disarm_always_wins_even_mid_mission():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    fsm.handle_command(MissionCommandCode.START)
    assert fsm.state == MissionState.MISSION_ACTIVE
    fsm.set_armed(False)
    assert fsm.state == MissionState.DISARMED


def test_start_rejected_when_disarmed():
    fsm = MissionStateMachine()
    result = fsm.handle_command(MissionCommandCode.START)
    assert not result.success
    assert fsm.state == MissionState.DISARMED


def test_full_start_stop_start_return_cycle():
    fsm = MissionStateMachine()
    fsm.set_armed(True)

    r1 = fsm.handle_command(MissionCommandCode.START)
    assert r1.success and fsm.state == MissionState.MISSION_ACTIVE

    r2 = fsm.handle_command(MissionCommandCode.STOP)
    assert r2.success and fsm.state == MissionState.MISSION_PAUSED

    r3 = fsm.handle_command(MissionCommandCode.START)
    assert r3.success and fsm.state == MissionState.MISSION_ACTIVE

    r4 = fsm.handle_command(MissionCommandCode.RETURN)
    assert r4.success and fsm.state == MissionState.RETURNING


def test_stop_rejected_unless_mission_active():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    result = fsm.handle_command(MissionCommandCode.STOP)
    assert not result.success
    assert fsm.state == MissionState.ARMED


def test_return_rejected_from_armed_idle():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    result = fsm.handle_command(MissionCommandCode.RETURN)
    assert not result.success
    assert fsm.state == MissionState.ARMED


def test_abort_works_from_mission_active():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    fsm.handle_command(MissionCommandCode.START)
    result = fsm.handle_command(MissionCommandCode.ABORT)
    assert result.success
    assert fsm.state == MissionState.RETURNING


def test_abort_works_even_during_fault():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    fsm.handle_command(MissionCommandCode.START)
    fsm.set_fault(True)
    assert fsm.state == MissionState.FAULT
    result = fsm.handle_command(MissionCommandCode.ABORT)
    assert result.success
    assert fsm.state == MissionState.RETURNING


def test_abort_rejected_when_disarmed():
    fsm = MissionStateMachine()
    result = fsm.handle_command(MissionCommandCode.ABORT)
    assert not result.success
    assert fsm.state == MissionState.DISARMED


def test_fault_clears_back_to_armed_not_to_previous_mission_state():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    fsm.handle_command(MissionCommandCode.START)
    fsm.set_fault(True)
    fsm.set_fault(False)
    # Operator must explicitly re-issue START; mission does not silently resume.
    assert fsm.state == MissionState.ARMED


def test_start_rejected_while_already_mission_active():
    fsm = MissionStateMachine()
    fsm.set_armed(True)
    fsm.handle_command(MissionCommandCode.START)
    result = fsm.handle_command(MissionCommandCode.START)
    assert not result.success
    assert fsm.state == MissionState.MISSION_ACTIVE
