# auv_vehicle — Vehicle State & Telemetry + Command Interface

Implements assessment items 1 and 2 (Software Head):
- Item 1: *"Develop a ROS 2 node that simulates and publishes essential AUV
  parameters."*
- Item 2: *"Implement an interface through which the AUV can receive
  commands"* (Arm/Disarm, Start/Stop/Return/Abort Mission, Set Target Depth).

These two components now live in one node, `vehicle_node` (see "Why one
node" below). The Ground Station, Simulation integration, and the full
Autonomy Head stack build on top of the interfaces defined here
(`auv_interfaces`) and will land as separate packages in the same workspace.

## What this package contains

```
auv_interfaces/                     # sibling package: shared interface definitions
  msg/VehicleState.msg
  msg/SystemStatus.msg
  msg/Heartbeat.msg
  srv/SetArmed.srv                  # Arm / Disarm
  srv/MissionCommand.srv            # Start / Stop / Return / Abort
  action/SetTargetDepth.action      # long-running, cancellable dive/rise

auv_vehicle/
  auv_vehicle/
    vehicle_sim_model.py            # pure-Python plant model, no rclpy dependency
    mission_state_machine.py        # pure-Python mission FSM, no rclpy dependency
    qos_profiles.py                 # shared QoS profiles, reused by later packages
    vehicle_node.py                 # THE node to launch: telemetry + commands
    telemetry_publisher.py          # telemetry-only node, kept for isolated testing
    gazebo_adapter.py               # item 4: same interface as vehicle_sim_model,
                                     # backed by a running Gazebo sim instead
  launch/telemetry.launch.py        # launches vehicle_node (see note below)
  test/test_vehicle_sim_model.py
  test/test_mission_state_machine.py
```

**Why one node, not two.** An earlier version of this package had a
telemetry-only node with no way to command it. Splitting command validation
into a separate node while state (`VehicleSimModel`) lives elsewhere would
mean either duplicating state (which drifts and lies) or adding an internal
topic just to keep two nodes' copies in sync — extra moving parts with no
real benefit at this scale. `vehicle_node` owns the model and the mission
state machine once, and exposes telemetry topics + command services/action
against that single source of truth. `launch/telemetry.launch.py` now
launches `vehicle_node` (a superset of the old telemetry-only behaviour),
so the launch command you already use keeps working unchanged.

## Topics published

| Topic            | Message                    | Rate (default) | QoS                                             |
|------------------|-----------------------------|-----------------|--------------------------------------------------|
| `/auv/telemetry` | `auv_interfaces/VehicleState` | 20 Hz          | BEST_EFFORT, VOLATILE, KEEP_LAST(5)               |
| `/auv/status`    | `auv_interfaces/SystemStatus` | 2 Hz           | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(1)           |
| `/auv/heartbeat` | `auv_interfaces/Heartbeat`    | 1 Hz           | RELIABLE, VOLATILE, KEEP_LAST(1)                  |

**Why three topics instead of one?** Each has a different reliability
contract:

- **Telemetry** is high-rate and stale samples are worthless the instant a
  fresher one exists, so we accept loss (`BEST_EFFORT`) in exchange for not
  backpressuring the publisher and not paying retransmission cost on what
  will eventually be a bandwidth-constrained acoustic/serial link to real
  hardware.
- **Status** is rare but must never be silently dropped, and a Ground
  Station reconnecting mid-mission must see the *current* status
  immediately — hence `RELIABLE` + `TRANSIENT_LOCAL` (the latched last
  message is delivered to new subscribers on connection).
- **Heartbeat** exists purely so the Ground Station can detect
  communication loss (item 3: *"Indicating communication/system
  failures"*) independently of whatever the last telemetry/status message
  said — a stale-but-nominal-looking status is itself a failure mode this
  topic is designed to catch. Full rationale is in `qos_profiles.py`.

## Command Interface (item 2)

| Interface                | Type    | Purpose                                    |
|---------------------------|---------|----------------------------------------------|
| `/auv/arm`                | Service (`SetArmed`)        | Arm / disarm                    |
| `/auv/mission_command`    | Service (`MissionCommand`)  | Start / Stop / Return / Abort   |
| `/auv/set_target_depth`   | Action (`SetTargetDepth`)   | Dive/rise to a depth, with feedback and cancellation |

**Why services for Arm/Disarm and mission commands, but an action for
depth:** the former are instantaneous — the caller needs a synchronous
accept/reject with a reason (e.g. "cannot arm: FAULT state"), and there is
no ongoing progress to report, so a topic (no acknowledgement) or an action
(needless overhead) would both be the wrong tool. Reaching a target depth,
by contrast, takes real, variable time; the operator benefits from
continuous feedback and must be able to cancel mid-dive — a service would
either block the caller for an unknown duration or have to fake a response
before the vehicle actually arrived.

Mission state machine (`MissionState`): `DISARMED → ARMED → MISSION_ACTIVE
⇄ MISSION_PAUSED`, with `RETURN`/`ABORT` moving to `RETURNING` from either
mission state, and a battery-triggered `FAULT` that can pre-empt any state
and falls back to `ARMED` (not the prior mission state) once cleared, so a
mission never silently resumes on its own. Full transition table and
rationale are in `mission_state_machine.py`; every transition is covered by
`test/test_mission_state_machine.py` (12 tests).

Example calls, once `vehicle_node` is running:

```bash
ros2 service call /auv/arm auv_interfaces/srv/SetArmed "{arm: true}"

ros2 service call /auv/mission_command auv_interfaces/srv/MissionCommand "{command: 0}"   # START
ros2 service call /auv/mission_command auv_interfaces/srv/MissionCommand "{command: 1}"   # STOP
ros2 service call /auv/mission_command auv_interfaces/srv/MissionCommand "{command: 2}"   # RETURN
ros2 service call /auv/mission_command auv_interfaces/srv/MissionCommand "{command: 3}"   # ABORT

ros2 action send_goal /auv/set_target_depth auv_interfaces/action/SetTargetDepth \
  "{target_depth: 10.0, tolerance: 0.15}" --feedback
```

## Parameters

| Parameter                | Default          | Description                                |
|----------------------------|------------------|-----------------------------------------------|
| `telemetry_rate_hz`        | `20.0`           | `/auv/telemetry` publish rate                 |
| `status_rate_hz`           | `2.0`            | `/auv/status` publish rate                    |
| `heartbeat_rate_hz`        | `1.0`            | `/auv/heartbeat` publish rate                 |
| `sim_seed`                 | `-1` (unseeded)  | RNG seed for simulated sensor noise           |
| `initial_armed`            | `false`          | Starting arm state                            |
| `frame_id`                 | `auv_base_link`  | Frame ID stamped in message headers           |
| `depth_tolerance_m`        | `0.15`           | Default `SetTargetDepth` success tolerance    |
| `depth_action_timeout_s`   | `60.0`           | Abort a stuck dive after this long            |
| `depth_feedback_period_s`  | `0.5`            | How often `SetTargetDepth` feedback is published |

## Simulation backend (item 4)

`vehicle_node` doesn't simulate anything itself — it delegates entirely to
whichever object satisfies `.step(dt) / .set_target_depth() /
.set_armed() / .state`. The `use_gazebo` parameter picks which one:

| `use_gazebo` | Backend                                    | Needs running          |
|--------------|----------------------------------------------|-------------------------|
| `false` (default) | `VehicleSimModel` — pure-Python plant model | nothing else            |
| `true`       | `GazeboVehicleAdapter` — commands/reads a real Gazebo sim | `ros2 launch auv_simulation simulation.launch.py` |

```bash
ros2 launch auv_simulation simulation.launch.py &
ros2 launch auv_vehicle telemetry.launch.py use_gazebo:=true initial_armed:=true sim_seed:=42
```

or, in one command: `ros2 launch auv_ground_station full_sim_stack.launch.py`.
See `auv_simulation/README.md` for what's actually in the Gazebo world and
why it's built the way it is.

## Design decisions

- **Simulation is decoupled from ROS wiring.** `VehicleSimModel` has no
  rclpy import and is unit-tested directly (`test/test_vehicle_sim_model.py`,
  6 tests, runnable with plain `pytest` — no ROS environment needed). The
  node's job is only parameters, timers, message construction and QoS.
  This is exactly what let the Simulation Environment item (above) land as
  a new backend behind the same interface rather than a rewrite: every
  QoS/message/parameter decision in this file was untouched by it.
- **`status` is duplicated on `VehicleState`** (mirroring the authoritative
  `/auv/status` channel) so a consumer that only needs telemetry doesn't
  also have to subscribe to the status topic, while `/auv/status` remains
  the source of truth for status *changes* thanks to its latched QoS.
- **Ground truth vs. noisy observation.** `VehicleSimModel.state` holds the
  noise-free internal state; `.step()` returns a separately-perturbed copy.
  This means later control/estimation code (Autonomy Head) can be pointed
  at the noisy stream while status logic (e.g. the battery fault threshold)
  reads ground truth, avoiding spurious fault-flapping caused by chasing
  the model's own injected noise.
- **Quaternion conversion is implemented locally** (`rpy_to_quaternion`)
  rather than depending on `tf_transformations`, to avoid a heavyweight
  dependency for one 15-line conversion.

## Build & run

From the workspace root (`AUV_int_task-2026/`):

```bash
source /opt/ros/<distro>/setup.bash     # e.g. humble, iron, jazzy
source AUV/bin/activate                 # project venv, if used for Python deps

colcon build --packages-select auv_interfaces auv_vehicle
source install/setup.bash

ros2 launch auv_vehicle telemetry.launch.py
# or, with an armed vehicle diving to 10 m and a fixed seed for reproducible demos:
ros2 launch auv_vehicle telemetry.launch.py initial_armed:=true sim_seed:=42
```

In another sourced terminal:

```bash
ros2 topic hz /auv/telemetry
ros2 topic echo /auv/status
ros2 topic echo /auv/heartbeat
```

## Running the unit tests without a ROS environment

```bash
cd src/auv_vehicle
python3 -m pytest test/test_vehicle_sim_model.py -v
```

## Known simplifications, worth naming in the writeup

- **Entering
  `FAULT` does not currently auto-trigger a return-to-surface; it only
  blocks new arming and forces the operator to explicitly re-`START`. A
  real system would likely auto-issue an internal ABORT on critical
  battery — noted here as a deliberate scope cut, not an oversight.
