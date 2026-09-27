"""
comms_monitor.py

Pure-Python communication-loss detector for the Ground Station (Software
Head, item 3: "Indicating communication/system failures"). Independent of
rclpy and Qt so it is unit-testable in isolation -- same split used for
VehicleSimModel and MissionStateMachine in auv_vehicle.

Detects comms loss purely from elapsed wall-clock time since the last
observed heartbeat, deliberately ignoring telemetry/status content: a
stream of stale-but-"nominal-looking" status messages is exactly the
failure mode a dedicated heartbeat channel exists to catch (see
auv_vehicle/qos_profiles.py for the corresponding publisher-side
reasoning). Checking the heartbeat's own freshness, rather than the
freshness of telemetry or status, means a comms failure is detected even
if the last thing received happened to look fine.
"""

from typing import Optional


class CommsMonitor:
    def __init__(self, timeout_s: float = 3.0):
        """timeout_s: comms are considered lost if no heartbeat has been
        recorded for this long. Should be set to a small multiple (e.g.
        3x) of the vehicle's heartbeat period, not the raw period itself,
        so ordinary network jitter doesn't false-trigger a "lost" state
        the instant one heartbeat arrives a little late."""
        self.timeout_s = timeout_s
        self._last_heartbeat_time: Optional[float] = None

    def on_heartbeat(self, now: float) -> None:
        self._last_heartbeat_time = now

    def is_lost(self, now: float) -> bool:
        if self._last_heartbeat_time is None:
            return True  # never received one -- treat as lost, not "unknown"
        return (now - self._last_heartbeat_time) > self.timeout_s

    def seconds_since_last_heartbeat(self, now: float) -> Optional[float]:
        if self._last_heartbeat_time is None:
            return None
        return now - self._last_heartbeat_time
