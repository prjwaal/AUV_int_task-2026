# auv_simulation — Simulation Environment

Implements assessment item 4 (Software Head): *"Integrate the software
stack with a simulated AUV using Gazebo, Webots, or another suitable
simulator. The simulation should demonstrate communication between the
simulated vehicle, ROS 2 nodes, and the ground station."*

Built on **Gazebo Sim (gz-sim) 8, "Harmonic"**, the version that ships and
is officially supported on Ubuntu 24.04 / ROS 2 Jazzy — matching this
workspace's `gz sim --version` (8.15.0).

## What this package contains

```
auv_simulation/
  worlds/auv_world.sdf          # seabed, waterline, lighting, buoyancy — the arena
  models/auv/model.sdf          # single-link torpedo hull, neutrally buoyant
  models/auv/model.config
  config/auv_bridge.yaml        # ros_gz_bridge topic mapping
  launch/simulation.launch.py   # gz sim + the bridge, nothing else
```

This package contains **no ROS 2 nodes**. It only stands up Gazebo and
the bridge that exposes it to ROS 2 as two topics:

| Topic                     | Direction        | Type                     |
|----------------------------|-------------------|---------------------------|
| `/model/auv/cmd_vel`       | ROS 2 → Gazebo    | `geometry_msgs/Twist`     |
| `/model/auv/odometry`      | Gazebo → ROS 2    | `nav_msgs/Odometry`       |
| `/clock`                   | Gazebo → ROS 2    | `rosgraph_msgs/Clock`     |

The ROS 2-side consumer of those two topics is `GazeboVehicleAdapter` in
`auv_vehicle` (see `auv_vehicle/auv_vehicle/gazebo_adapter.py`), not
anything in this package.

## Design decisions

**Velocity control, not a thruster/fin allocation model.** The vehicle's
single link is driven by Gazebo's built-in
`gz::sim::systems::VelocityControl` system: it accepts a body-frame
`Twist` on `/model/auv/cmd_vel` and applies it directly as the link's
velocity. A physically faithful AUV has 4-8 thrusters and a
force/torque allocation matrix mapping desired body velocity to individual
thruster commands — genuinely useful for the *Autonomy Head* control-system
item, but out of scope for what the Software Head assessment is actually
evaluating here (item 4 asks for "integrate the software stack with a
simulated AUV" and "demonstrate communication", not a hydrodynamics
model). `VelocityControl` gives real physics-engine integration — the
vehicle has mass, inertia, and buoyancy, and collides with the seabed —
while keeping the actuation interface a single clean boundary that a real
thruster allocator slots into later without changing anything upstream of
it (see "Future hardware integration" below).

**Buoyancy, not a fixed/kinematic body.** `gz-sim-buoyancy-system` is
attached to the world with `uniform_fluid_density = 1000` (fresh water).
The hull's mass (`models/auv/model.sdf`) is set to
`volume × 1000 kg/m³`, i.e. neutrally buoyant: under gravity alone the
vehicle neither sinks nor rises, and only moves because
`GazeboVehicleAdapter` is commanding it — consistent with the
"disarmed vehicle doesn't move" behaviour `VehicleSimModel` already
established for the non-Gazebo backend, and cheap to get from a stock
Gazebo system rather than hand-rolling it.

**Ground truth from `OdometryPublisher`, not a custom pose-reporting
plugin.** `gz::sim::systems::OdometryPublisher` (`dimensions: 3`) already
publishes exactly the pose+twist message shape needed
(`nav_msgs/Odometry`), at a configurable rate, over Gazebo Transport —
bridging a stock message type needs no custom `.proto`/message-conversion
code in `ros_gz_bridge`, unlike a bespoke telemetry topic would.

**One-way bridging.** `auv_bridge.yaml` bridges `cmd_vel` ROS→GZ only and
`odometry`/`clock` GZ→ROS only, rather than bidirectionally. Nothing on
the Gazebo side ever needs to read `cmd_vel` back, and nothing on the ROS
side ever publishes odometry — bidirectional bridging would silently
double the bridge's subscription/advertisement count for no consumer.

**No sensor plugins (camera/sonar/IMU) in this world.** Those matter for
the *Autonomy Head* perception item; the Software Head assessment's
Simulation Environment item only asks for vehicle/ground-station
communication through a simulated vehicle. Adding them is a small,
additive change to `models/auv/model.sdf` (a `<sensor>` block per sensor)
whenever they're needed — flagged here as a clean seam, not a gap.

## Future hardware integration

The `/model/auv/cmd_vel` in / `/model/auv/odometry` out boundary in this
package is deliberately the same shape a real vehicle's low-level control
board would present: "accept a body-frame velocity/thruster-mix command,
report back pose and velocity from onboard sensors/state estimation."
Moving from simulation to real hardware means replacing
`GazeboVehicleAdapter` with a `HardwareVehicleAdapter` that talks to actual
thruster drivers and a real state estimator (DVL/IMU/depth sensor fusion)
behind the exact same `.step()/.set_target_depth()/.set_armed()/.state`
interface — `vehicle_node.py`, the command interface, the mission state
machine, and the Ground Station do not change at all.

## Build & run

From the workspace root:

```bash
colcon build --packages-select auv_interfaces auv_vehicle auv_simulation auv_ground_station
source install/setup.bash

# Everything at once: Gazebo + bridge + vehicle_node(use_gazebo:=true) + ground station
ros2 launch auv_ground_station full_sim_stack.launch.py initial_armed:=true sim_seed:=42

# Or the simulator on its own (e.g. to watch it while iterating on vehicle_node
# in another terminal):
ros2 launch auv_simulation simulation.launch.py
# ...then in another sourced terminal:
ros2 launch auv_vehicle telemetry.launch.py use_gazebo:=true initial_armed:=true
```

Sanity-check the bridge directly, without any ROS 2 node running:

```bash
ros2 topic echo /model/auv/odometry --once
ros2 topic pub --once /model/auv/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.15}}"
```

If `ros2 topic echo /model/auv/odometry` shows nothing, `gz topic -l` will
say whether Gazebo itself is publishing `/model/auv/odometry` on the
Gazebo Transport side (a modelling/plugin problem) or whether the topic
just isn't there at all (the bridge isn't running, or
`GZ_SIM_RESOURCE_PATH` didn't resolve `model://auv` — check the `gz sim`
terminal output for `[Err] ... Unable to find uri[model://auv]`).

## Known simplifications, worth naming in the writeup

- **Not hydrodynamically accurate.** No added mass, no drag/damping model,
  no current or wave disturbance. The vehicle tracks commanded velocity
  almost exactly, which is intentional for a controllable, demo-friendly
  simulation, but is not a substitute for a Fossen-equations hydrodynamics
  model (`gz-sim` ships one — `gz::sim::systems::Hydrodynamics` — were this
  the Autonomy Head Control System item instead).
- **Battery is still a software model** (`GazeboVehicleAdapter` reuses
  `VehicleSimModel`'s drain formula), since Gazebo has no battery physics
  wired into this world.
- **Flat seabed, no obstacles/targets.** Sufficient for demonstrating
  vehicle↔ROS 2↔Ground Station communication (this package's actual
  scope); a mission-relevant target (gate/buoy/marker) is an Autonomy Head
  concern.
