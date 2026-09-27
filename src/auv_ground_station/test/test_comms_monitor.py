"""
Unit tests for CommsMonitor. Pure Python, no rclpy/Qt import -- runnable
with plain pytest.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from auv_ground_station.comms_monitor import CommsMonitor  # noqa: E402


def test_lost_when_never_received():
    monitor = CommsMonitor(timeout_s=3.0)
    assert monitor.is_lost(now=100.0) is True


def test_seconds_since_last_heartbeat_none_when_never_received():
    monitor = CommsMonitor(timeout_s=3.0)
    assert monitor.seconds_since_last_heartbeat(now=100.0) is None


def test_not_lost_immediately_after_heartbeat():
    monitor = CommsMonitor(timeout_s=3.0)
    monitor.on_heartbeat(now=100.0)
    assert monitor.is_lost(now=100.1) is False


def test_not_lost_just_under_timeout():
    monitor = CommsMonitor(timeout_s=3.0)
    monitor.on_heartbeat(now=100.0)
    assert monitor.is_lost(now=102.9) is False


def test_lost_after_timeout_elapses():
    monitor = CommsMonitor(timeout_s=3.0)
    monitor.on_heartbeat(now=100.0)
    assert monitor.is_lost(now=103.1) is True


def test_recovers_after_new_heartbeat():
    monitor = CommsMonitor(timeout_s=3.0)
    monitor.on_heartbeat(now=100.0)
    assert monitor.is_lost(now=104.0) is True
    monitor.on_heartbeat(now=104.5)
    assert monitor.is_lost(now=105.0) is False


def test_seconds_since_last_heartbeat_reports_elapsed_time():
    monitor = CommsMonitor(timeout_s=3.0)
    monitor.on_heartbeat(now=50.0)
    assert monitor.seconds_since_last_heartbeat(now=52.5) == 2.5
