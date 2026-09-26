"""
mission_state_machine.py

Pure-Python mission-status state machine for the Vehicle Command Interface
(Software Head, item 2). Kept independent of rclpy, following the same
split used for vehicle_sim_model.py: ROS nodes call into this class and
publish its results; they do not reimplement its transition logic. See
test/test_mission_state_machine.py for unit tests that exercise every
transition without needing a ROS 2 environment.

States mirror the status codes shared across auv_interfaces/msg
(VehicleState, SystemStatus) and the two command services
(SetArmed, MissionCommand):
    DISARMED (0), ARMED (1), MISSION_ACTIVE (2), MISSION_PAUSED (3),
    RETURNING (4), FAULT (5)

DISARMED and FAULT are vehicle-level conditions rather than mission
commands, so they are applied directly via set_armed()/set_fault() and can
pre-empt or unlock whatever a mission command would otherwise do.
"""

from dataclasses import dataclass
from enum import IntEnum


class MissionState(IntEnum):
    DISARMED = 0
    ARMED = 1
    MISSION_ACTIVE = 2
    MISSION_PAUSED = 3
    RETURNING = 4
    FAULT = 5


class MissionCommandCode(IntEnum):
    START = 0
    STOP = 1
    RETURN = 2
    ABORT = 3


@dataclass
class CommandResult:
    success: bool
    message: str
    resulting_state: MissionState


class MissionStateMachine:
    """Tracks mission-level state and validates mission-command transitions."""

    def __init__(self):
        self._state = MissionState.DISARMED

    @property
    def state(self) -> MissionState:
        return self._state

    def set_armed(self, armed: bool) -> None:
        """Arm/disarm the vehicle. Disarming always wins and cancels
        whatever mission state was active -- it is the fail-safe
        direction and must never be blocked by mission logic."""
        if armed:
            if self._state == MissionState.DISARMED:
                self._state = MissionState.ARMED
            # else: already armed/mid-mission/fault -- arming again is a no-op
        else:
            self._state = MissionState.DISARMED

    def set_fault(self, active: bool) -> None:
        """Enter/exit FAULT independent of mission commands (e.g. driven
        by a battery threshold check on a timer)."""
        if active:
            self._state = MissionState.FAULT
        elif self._state == MissionState.FAULT:
            # Fault cleared -- fall back to ARMED; the operator must
            # explicitly re-issue START to resume a mission rather than
            # having one silently resume on its own.
            self._state = MissionState.ARMED

    def handle_command(self, command: MissionCommandCode) -> CommandResult:
        handlers = {
            MissionCommandCode.START: self._handle_start,
            MissionCommandCode.STOP: self._handle_stop,
            MissionCommandCode.RETURN: self._handle_return,
            MissionCommandCode.ABORT: self._handle_abort,
        }
        return handlers[command]()

    def _handle_start(self) -> CommandResult:
        if self._state in (MissionState.ARMED, MissionState.MISSION_PAUSED):
            self._state = MissionState.MISSION_ACTIVE
            return CommandResult(True, "Mission started", self._state)
        return CommandResult(
            False, f"Cannot start mission from state {self._state.name}", self._state
        )

    def _handle_stop(self) -> CommandResult:
        if self._state == MissionState.MISSION_ACTIVE:
            self._state = MissionState.MISSION_PAUSED
            return CommandResult(True, "Mission paused", self._state)
        return CommandResult(
            False, f"Cannot stop mission from state {self._state.name}", self._state
        )

    def _handle_return(self) -> CommandResult:
        if self._state in (MissionState.MISSION_ACTIVE, MissionState.MISSION_PAUSED):
            self._state = MissionState.RETURNING
            return CommandResult(True, "Returning to home", self._state)
        return CommandResult(
            False, f"Cannot return from state {self._state.name}", self._state
        )

    def _handle_abort(self) -> CommandResult:
        # Abort is a safety command: permissive from any armed state,
        # including mid-return. The only state that legitimately blocks it
        # is DISARMED (nothing to abort).
        if self._state == MissionState.DISARMED:
            return CommandResult(
                False, "Cannot abort: vehicle is disarmed", self._state
            )
        self._state = MissionState.RETURNING
        return CommandResult(True, "Mission aborted, returning to home", self._state)
