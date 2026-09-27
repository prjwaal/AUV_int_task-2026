# auv_ground_station — Ground Station GUI

Implements assessment item 3 (Software Head): *"Develop a basic
ground-station interface capable of monitoring live vehicle telemetry,
displaying vehicle status, displaying depth and position, sending commands
to the AUV, and indicating communication/system failures."*

Built with **PySide6** (Qt for Python) — the framework most AUV/ROV
competition teams (RoboSub, SAUVC, etc.) actually use for a custom ground
station: pure Python (same language as the ROS 2 nodes, no separate web
server/rosbridge layer), and a natural fit for the polished, judge-facing
demo video this assessment asks for.

## What this package contains

```
auv_ground_station/
  comms_monitor.py     # pure-Python heartbeat-staleness detector, no rclpy/Qt dependency
  ros_bridge.py         # GroundStationNode (rclpy) + RosBridge (Qt signal hub)
  main_window.py         # PySide6 UI: telemetry, status/comms badges, commands, log
  app.py                 # entry point: starts the ROS thread + Qt event loop
launch/
  ground_station.launch.py   # just the ground station (vehicle must already be running)
  full_stack.launch.py       # vehicle_node (VehicleSimModel) + ground_station, one command
  full_sim_stack.launch.py   # Gazebo + bridge + vehicle_node (Gazebo backend) + ground_station
test/
  test_comms_monitor.py      # 7 unit tests, no ROS/Qt needed
```

## Install the one extra dependency

PySide6 isn't a ROS package — install it once with pip:

```bash
pip install PySide6 --break-system-packages
# or, without that flag, inside a --system-site-packages venv
```

## Threading model (read this before touching ros_bridge.py)

`rclpy` and Qt each want to own their own event loop, so this node splits
across two threads (see `app.py`):

- **Background thread**: runs a plain `while rclpy.ok(): executor.spin_once()`
  loop. All rclpy calls — subscriptions, service/action clients — happen
  only here, avoiding rclpy's general lack of guarantees about concurrent
  calls from multiple threads.
- **Main thread**: runs the Qt event loop (`app.exec()`) and owns every
  widget.

**ROS → GUI** (telemetry, status, comms, command results) goes through Qt
signals emitted from the background thread. This works safely with no
extra locking: Qt delivers a signal based on the *receiving* object's
thread affinity, and since `MainWindow` was constructed on the main thread,
PySide6 automatically queues these cross-thread emissions onto the main
thread's event loop.

**GUI → ROS** (button clicks) deliberately does **not** rely on that same
mechanism, because automatic queued delivery requires the *receiving*
thread to be running a Qt event loop — and the background thread here runs
a plain rclpy spin loop, not one. Instead, button handlers put a plain
tuple on `RosBridge.command_queue` (a thread-safe `queue.Queue`); the
background loop drains it every 50ms and calls `GroundStationNode
.handle_command()`. This is simpler and more robust than forcing Qt's
cross-thread signal machinery to work into a thread it wasn't designed
to talk to.

## Comms-failure indication (item 3, explicitly)

`CommsMonitor` (pure Python, unit-tested — `test/test_comms_monitor.py`,
7 tests) flags comms as lost purely from elapsed time since the last
`/auv/heartbeat` message, deliberately *not* from whether telemetry/status
still look "fine" — a stream of stale-but-nominal-looking status messages
is exactly the failure mode a dedicated heartbeat channel exists to catch.
Timeout defaults to 3× the vehicle's heartbeat period (parameterised, not
hardcoded) so ordinary jitter doesn't false-trigger.

## Build & run

```bash
cd ~/AUV_int_task-2026
colcon build --packages-select auv_interfaces auv_vehicle auv_ground_station
source install/setup.bash

# one command, brings up both the vehicle (VehicleSimModel) and the ground station:
ros2 launch auv_ground_station full_stack.launch.py initial_armed:=true sim_seed:=42

# one command, brings up Gazebo + the vehicle (Gazebo-backed) + the ground station
# (requires auv_simulation -- see its README for what's actually simulated):
ros2 launch auv_ground_station full_sim_stack.launch.py initial_armed:=true

# or, if the vehicle is already running elsewhere:
ros2 launch auv_ground_station ground_station.launch.py
```

The GUI window shows live depth/position/velocity/battery, a colour-coded
status badge, a colour-coded comms badge, Arm/Disarm buttons, Start/Stop/
Return/Abort buttons, a target-depth spinbox with Go/Cancel, and a
scrolling log of every command's result — good material for the 3–5 minute
demo video this assessment asks for.

## Running the unit tests without ROS or a display

```bash
cd src/auv_ground_station
python3 -m pytest test/test_comms_monitor.py -v
```

## Design decision worth calling out in the writeup

`main_window.py` imports neither `rclpy` nor anything from `ros_bridge.py`
directly — it only knows about `RosBridge`'s signals and its
`command_queue`. This means the UI can be developed and tested (as it was
here, rendered offscreen with `QT_QPA_PLATFORM=offscreen`) without a ROS
environment or a real display at all.
