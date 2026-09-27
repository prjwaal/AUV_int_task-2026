"""
main_window.py

PySide6 main window for the Ground Station (Software Head, item 3):
  - Monitoring live vehicle telemetry
  - Displaying vehicle status
  - Displaying depth and position
  - Sending commands to the AUV
  - Indicating communication/system failures

This module never imports rclpy and never blocks. Every button handler
just enqueues a command tuple on `RosBridge.command_queue` and returns
immediately; every displayed value arrives asynchronously via a Qt signal
emitted from the ROS thread (see ros_bridge.py for the full threading
rationale).
"""

import time

from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

STATUS_NAMES = {
    0: "DISARMED",
    1: "ARMED",
    2: "MISSION ACTIVE",
    3: "MISSION PAUSED",
    4: "RETURNING",
    5: "FAULT",
}
STATUS_COLORS = {
    0: "#888888",
    1: "#2b7de9",
    2: "#2ecc71",
    3: "#f1c40f",
    4: "#e67e22",
    5: "#e74c3c",
}

MISSION_START, MISSION_STOP, MISSION_RETURN, MISSION_ABORT = 0, 1, 2, 3

_BADGE_STYLE = "font-weight: bold; padding: 6px; border-radius: 3px; color: white; background: {color};"


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self._bridge = bridge
        self.setWindowTitle("AUV Ground Station")
        self.resize(780, 540)

        self._build_ui()
        self._connect_signals()

    # ------------------------------------------------------------------ #
    # UI construction
    # ------------------------------------------------------------------ #

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)

        root.addLayout(self._build_header())

        mid = QHBoxLayout()
        mid.addWidget(self._build_telemetry_group(), 2)
        mid.addWidget(self._build_command_group(), 1)
        root.addLayout(mid)

        root.addWidget(self._build_log_group())

        self.setCentralWidget(central)

    def _build_header(self):
        layout = QHBoxLayout()

        self.comms_label = QLabel("COMMS: UNKNOWN")
        self.comms_label.setStyleSheet(_BADGE_STYLE.format(color="#888888"))
        layout.addWidget(self.comms_label)

        self.status_label = QLabel("STATUS: --")
        self.status_label.setStyleSheet(_BADGE_STYLE.format(color="#888888"))
        layout.addWidget(self.status_label)

        self.status_message_label = QLabel("")
        layout.addWidget(self.status_message_label, 1)

        return layout

    def _build_telemetry_group(self):
        box = QGroupBox("Telemetry")
        grid = QGridLayout(box)

        self._telemetry_labels = {}
        fields = [
            ("depth", "Depth (m)"),
            ("position", "Position — x, y, z (m)"),
            ("velocity", "Velocity — vx, vy, vz (m/s)"),
            ("battery_percentage", "Battery (%)"),
            ("battery_voltage", "Battery (V)"),
        ]
        for row, (key, label) in enumerate(fields):
            grid.addWidget(QLabel(label + ":"), row, 0)
            value_label = QLabel("--")
            self._telemetry_labels[key] = value_label
            grid.addWidget(value_label, row, 1)

        grid.setRowStretch(len(fields), 1)
        return box

    def _build_command_group(self):
        box = QGroupBox("Commands")
        layout = QVBoxLayout(box)

        arm_row = QHBoxLayout()
        arm_btn = QPushButton("Arm")
        disarm_btn = QPushButton("Disarm")
        arm_btn.clicked.connect(lambda: self._send_arm(True))
        disarm_btn.clicked.connect(lambda: self._send_arm(False))
        arm_row.addWidget(arm_btn)
        arm_row.addWidget(disarm_btn)
        layout.addLayout(arm_row)

        mission_row = QHBoxLayout()
        for label, code in [
            ("Start", MISSION_START),
            ("Stop", MISSION_STOP),
            ("Return", MISSION_RETURN),
            ("Abort", MISSION_ABORT),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _checked=False, c=code: self._send_mission(c))
            mission_row.addWidget(btn)
        layout.addLayout(mission_row)

        depth_row = QHBoxLayout()
        self.depth_spin = QDoubleSpinBox()
        self.depth_spin.setRange(0.0, 100.0)
        self.depth_spin.setValue(10.0)
        self.depth_spin.setSuffix(" m")
        go_btn = QPushButton("Go to depth")
        cancel_btn = QPushButton("Cancel dive")
        go_btn.clicked.connect(self._send_set_depth)
        cancel_btn.clicked.connect(
            lambda: self._bridge.command_queue.put(("cancel_depth",))
        )
        depth_row.addWidget(self.depth_spin)
        depth_row.addWidget(go_btn)
        depth_row.addWidget(cancel_btn)
        layout.addLayout(depth_row)

        self.depth_progress_label = QLabel("")
        layout.addWidget(self.depth_progress_label)

        layout.addStretch(1)
        return box

    def _build_log_group(self):
        box = QGroupBox("Log")
        layout = QVBoxLayout(box)
        self.log_list = QListWidget()
        layout.addWidget(self.log_list)
        return box

    # ------------------------------------------------------------------ #
    # Signal wiring (ROS -> GUI)
    # ------------------------------------------------------------------ #

    def _connect_signals(self):
        self._bridge.telemetry_received.connect(self._on_telemetry)
        self._bridge.status_received.connect(self._on_status)
        self._bridge.comms_status_changed.connect(self._on_comms_status)
        self._bridge.command_result.connect(self._on_command_result)
        self._bridge.depth_feedback.connect(self._on_depth_feedback)
        self._bridge.depth_action_result.connect(self._on_depth_result)

    def _on_telemetry(self, data: dict):
        self._telemetry_labels["depth"].setText(f"{data['depth']:.2f}")
        self._telemetry_labels["position"].setText(
            f"{data['x']:.2f}, {data['y']:.2f}, {data['z']:.2f}"
        )
        self._telemetry_labels["velocity"].setText(
            f"{data['vx']:.2f}, {data['vy']:.2f}, {data['vz']:.2f}"
        )
        self._telemetry_labels["battery_percentage"].setText(
            f"{data['battery_percentage']:.1f}"
        )
        self._telemetry_labels["battery_voltage"].setText(
            f"{data['battery_voltage']:.2f}"
        )

    def _on_status(self, data: dict):
        status = data["status"]
        name = STATUS_NAMES.get(status, f"UNKNOWN({status})")
        color = STATUS_COLORS.get(status, "#888888")
        self.status_label.setText(f"STATUS: {name}")
        self.status_label.setStyleSheet(_BADGE_STYLE.format(color=color))
        self.status_message_label.setText(data["status_message"])

    def _on_comms_status(self, ok: bool):
        if ok:
            self.comms_label.setText("COMMS: OK")
            self.comms_label.setStyleSheet(_BADGE_STYLE.format(color="#2ecc71"))
        else:
            self.comms_label.setText("COMMS: LOST")
            self.comms_label.setStyleSheet(_BADGE_STYLE.format(color="#e74c3c"))
        self._log(f"Comms {'restored' if ok else 'LOST'}")

    def _on_command_result(self, name: str, success: bool, message: str):
        outcome = "OK" if success else "REJECTED"
        self._log(f"[{name}] {outcome}: {message}")

    def _on_depth_feedback(self, current_depth: float, distance_remaining: float):
        self.depth_progress_label.setText(
            f"Diving: {current_depth:.2f} m (Δ{distance_remaining:.2f} m)"
        )

    def _on_depth_result(self, success: bool, final_depth: float, message: str):
        self.depth_progress_label.setText(
            f"{message} (final depth {final_depth:.2f} m)"
        )
        self._log(f"[set_target_depth] {'OK' if success else 'FAILED'}: {message}")

    # ------------------------------------------------------------------ #
    # Command handlers (GUI -> ROS, via command_queue only -- see
    # ros_bridge.py for why this never calls rclpy directly)
    # ------------------------------------------------------------------ #

    def _send_arm(self, arm: bool):
        self._bridge.command_queue.put(("arm", arm))

    def _send_mission(self, code: int):
        self._bridge.command_queue.put(("mission", code))

    def _send_set_depth(self):
        target = self.depth_spin.value()
        self._bridge.command_queue.put(("set_depth", target, 0.15))

    def _log(self, text: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_list.addItem(f"[{timestamp}] {text}")
        self.log_list.scrollToBottom()
