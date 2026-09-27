"""
app.py

Entry point for the Ground Station GUI (Software Head, item 3).

Usage:
    ros2 run auv_ground_station ground_station
    ros2 launch auv_ground_station full_stack.launch.py   # also starts the vehicle
"""

import sys
import threading

import rclpy
from PySide6.QtWidgets import QApplication
from rclpy.executors import SingleThreadedExecutor

from auv_ground_station.main_window import MainWindow
from auv_ground_station.ros_bridge import GroundStationNode, RosBridge


def _ros_spin_loop(
    node: GroundStationNode,
    executor: SingleThreadedExecutor,
    bridge: RosBridge,
    stop_event: threading.Event,
):
    """Runs entirely on the background thread: spins the executor and
    drains queued GUI commands. See ros_bridge.py for why all rclpy calls
    are confined to this one thread."""
    while rclpy.ok() and not stop_event.is_set():
        executor.spin_once(timeout_sec=0.05)
        while not bridge.command_queue.empty():
            try:
                command = bridge.command_queue.get_nowait()
            except Exception:  # noqa: BLE001
                break
            node.handle_command(command)


def main():
    rclpy.init(args=sys.argv)

    bridge = RosBridge()  # created on the main thread
    node = GroundStationNode(bridge)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_event = threading.Event()
    ros_thread = threading.Thread(
        target=_ros_spin_loop,
        args=(node, executor, bridge, stop_event),
        daemon=True,
    )
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(bridge)
    window.show()

    exit_code = app.exec()

    stop_event.set()
    ros_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
