# AUV Software & Autonomy Stack — Technical Writeup

**Workspace:** `AUV_int_task-2026` · **Platform:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.15.0 (Harmonic)
**Covers:** Software Head (all five items) and the Autonomy Head items completed so far
(Navigation & Localization, Computer Vision, Control System).

> This document is written to be checked, not just read. Every claim is tagged with how
> it was established (see §1.2), and §9 lists what is *not* yet verified or *not yet built*.
> Nothing here should be read as "works" unless §1 says who ran it.

---

## Contents

1. Scope, status and how each claim was verified
2. System architecture
3. Cross-cutting design principles
4. Software Head
   4.1 Interfaces · 4.2 Telemetry and QoS · 4.3 Command interface · 4.4 Ground station · 4.5 Simulation
5. Autonomy Head
   5.1 Navigation & Localization · 5.2 Control System · 5.3 Computer Vision
6. Testing strategy and results
7. Defects found and fixed during development
8. Assumptions
9. Limitations, known gaps and unverified items
10. Remaining work
11. Mapping to the assessment criteria
Appendix A — Interface reference · Appendix B — Package layout · Appendix C — Verification checklist

---

## 1. Scope, status and how each claim was verified

### 1.1 What exists

| Component | Assessment item | State |
|---|---|---|
| Vehicle state & telemetry | Software Head 1 | Implemented |
| Vehicle command interface | Software Head 2 | Implemented |
| Ground station (PySide6) | Software Head 3 | Implemented |
| Gazebo simulation integration | Software Head 4 | Implemented |
| System architecture | Software Head 5 | This document + the workspace README diagram (autonomy packages not yet reflected in the README diagram) |
| Navigation & Localization | Autonomy Head 1 | Implemented |
| Computer Vision | Autonomy Head 2 | Implemented |
| Motion Planning | Autonomy Head 3 | **Not started** |
| Control System | Autonomy Head 4 | Implemented |
| Autonomous Mission Management | Autonomy Head 5 | **Not started** |
| Repository README, demo video, final submission packaging | Both | **Not started** |

### 1.2 Verification levels (used throughout)

| Tag | Meaning |
|---|---|
| **[U]** | Unit-tested in isolation; tests executed and passing (68 tests, §6). |
| **[R]** | Confirmed running on the developer's machine (build succeeded; output pasted back). |
| **[D]** | Checked against official documentation for the exact installed version (Gazebo 8.15.0 / ROS 2 Jazzy). |
| **[S]** | Checked by simulation or calculation in the development sandbox (numbers reported are measured there). |
| **[?]** | **Not verified.** Written carefully, but never executed in the environment that matters. |

### 1.3 Honest status matrix

| Area | Verified how |
|---|---|
| `colcon build` of interfaces + vehicle + ground station | **[R]** |
| Telemetry/status/heartbeat publishing; status values, battery drain rate | **[R]** (the `/auv/status` output was inspected and the drain arithmetic checked against the configured rate) |
| Ground station window launches and both nodes run together | **[R]** (after fixing the Python-environment and `xcb-cursor0` issues, §7) |
| Mission state machine, plant model, comms monitor logic | **[U]** |
| Command services/action end-to-end from the GUI | **[?]** code-reviewed; not reported back as exercised |
| Gazebo world, model, bridge, adapter | **[D]** plugin names, topics and YAML schema checked against Gazebo docs; **[?]** never executed by the author of this document, and not yet reported back as run |
| Navigation / control / perception *logic* | **[U]** and **[S]** |
| Navigation / control / perception *ROS nodes* and the Gazebo camera | **[?]** — syntax-checked only; `rclpy` is not available in the development sandbox |
| RViz path display, `rqt_image_view` overlay | **[?]** |

The single most important consequence: **the new autonomy packages have never been run
against ROS.** They are unit-tested at the level of their pure logic and nothing above it.
§7 (defect #11) shows why that gap is not academic.

---

## 2. System architecture

### 2.1 Layered view

```
┌────────────────────────────────────────────────────────────────────────────────┐
│ GROUND STATION (PySide6)                                                         │
│   telemetry · status badge · comms badge · commands · log                        │
└───────▲───────────────────────────────────────────────┬──────────────────────────┘
        │ /auv/telemetry (best-effort)                  │ services + action
        │ /auv/status    (reliable, latched)            │ /auv/arm, /auv/mission_command,
        │ /auv/heartbeat (reliable)                     │ /auv/set_target_depth
┌───────┴───────────────────────────────────────────────▼──────────────────────────┐
│ VEHICLE LAYER — vehicle_node (single owner of vehicle state)                     │
│   MissionStateMachine · telemetry timers · command handlers                      │
│   plant backend (chosen by `use_gazebo`):                                        │
│     VehicleSimModel  (pure Python)   or   GazeboVehicleAdapter                   │
└───────▲──────────────────────────────────────────────┬───────────────────────────┘
        │ /auv/control/cmd_vel (Twist, optional override)│ /model/auv/cmd_vel
        │                                               │ /model/auv/odometry, /clock, /camera
┌───────┴──────────────────┐                  ┌─────────▼───────────────────────────┐
│ AUTONOMY                  │                  │ SIMULATOR — Gazebo Sim 8 + ros_gz_bridge│
│  auv_control  (PID)       │                  │  VelocityControl · Buoyancy ·           │
│      ▲ NavigationState     │                  │  OdometryPublisher · camera sensor      │
│  auv_navigation (filter,   │                  │  seabed · waterline · target buoy       │
│      errors, path)         │                  └───────────────────────────────────────┘
│      ▲ /auv/telemetry      │                                   │ /camera
│  auv_perception (vision) ◄─┼───────────────────────────────────┘
│      ► /auv/vision/target_detection                                                  │
│  [not built: planning, mission management]                                           │
└───────────────────────────┘
```

### 2.2 Packages

| Package | Type | Role |
|---|---|---|
| `auv_interfaces` | ament_cmake | Messages, services, action shared by everything else |
| `auv_vehicle` | ament_python | `vehicle_node`, plant backends, mission FSM, QoS profiles |
| `auv_ground_station` | ament_python | PySide6 GUI + ROS bridge |
| `auv_simulation` | ament_cmake | Gazebo world/model/bridge config/launch. Contains **no nodes** |
| `auv_navigation` | ament_python | Filtered estimate, target, errors, trajectory |
| `auv_control` | ament_python | PID depth + heading |
| `auv_perception` | ament_python | Colour-blob vision |
| `auv_bringup` | ament_python | Launch files only |

`auv_interfaces` is deliberately a leaf: every other package depends on it and it depends
on nothing of ours, so no dependency cycle is possible through it. `auv_simulation` and
`auv_bringup` are deliberately node-free so that composition and environment
configuration never leak into logic packages.

### 2.3 Data flow of the closed autonomy loop

```
vehicle_node ──/auv/telemetry──► navigation_node ──/auv/navigation/state──► control_node
   ▲  (noisy, 20 Hz)              (filter + errors)        (depth_error, heading_error)   │
   │                                                                                       │
   └────────────────── /auv/control/cmd_vel (vx, vz, wz) ◄────────────────────────────────┘
```

Three properties are worth stating because they are easy to get wrong:

1. **Navigation owns error calculation; control never sees the target.** The assessment
   lists "position/heading error calculation" under Navigation. `NavigationState`
   therefore carries `depth_error` and `heading_error`, and `auv_control` consumes only
   those. This makes the control package trivially reusable and its tests independent of
   geometry.
2. **The controller acts on the *filtered* estimate, not the raw stream.** Filter lag is
   therefore inside the control loop (quantified in §5.1.4, and included in the tuning
   simulation in §5.2.3).
3. **The loop closes through `vehicle_node`, not around it.** `auv_control` never talks to
   Gazebo or to a plant model; it publishes one topic. Which backend is underneath is
   invisible to it.

---

## 3. Cross-cutting design principles

These recur in every component and explain most of the structure.

**P1 — Pure core, thin ROS shell.** Every piece of logic that can be free of `rclpy` is.
`VehicleSimModel`, `MissionStateMachine`, `CommsMonitor`, `StateEstimator`,
`navigation_math`, `PIDController`, `ColorBlobDetector` and `image_conversion` import no
ROS. ROS nodes contain parameter declaration, timers, message construction and QoS, and
nothing else. *Consequence:* the logic could be tested in a sandbox with no ROS install
(68 tests), and *cost:* the ROS wiring itself is the untested layer (§6.3).

**P2 — One owner per piece of state.** `vehicle_node` owns vehicle state, the mission FSM
and the plant. An earlier telemetry-only node was merged into it precisely because
commands must change the same state that telemetry reports; splitting them would have
required duplicated state or an internal sync topic for no benefit at this scale.

**P3 — Named seams for known future replacements.** `use_gazebo` swaps the plant.
`EstimatedState`/`update()` is where an EKF would replace the filter.
`Detection` is where a learned detector would replace HSV thresholding. Each seam is
documented at the point of the simplification, so a simplification is never mistaken
for a claim.

**P4 — Override with staleness fallback.** The Control System's output is an *optional
override* on the plant, valid for 1 s after the last message. If `auv_control` is absent,
crashed or unlaunched, the vehicle reverts to its earlier behaviour rather than freezing
on a stale command. This also kept every pre-autonomy demo and test working unchanged.

**P5 — Say what a simplification is.** Each documented simplification names what a real
system would do. This is deliberate: the assessment weights the ability to justify
engineering decisions, and a known, named limitation is a stronger position than an
unexamined one.

---

## 4. Software Head

### 4.1 Interfaces (`auv_interfaces`)

| Type | Name | Purpose |
|---|---|---|
| msg | `VehicleState` | Pose, depth, velocities, battery, status; 20 Hz |
| msg | `SystemStatus` | Status code, battery, human-readable message |
| msg | `Heartbeat` | Sequence number for liveness |
| msg | `NavigationState` | Filtered estimate, target, errors |
| msg | `TargetDetection` | Vision output |
| srv | `SetArmed` | Arm/disarm with accept/reject and reason |
| srv | `MissionCommand` | Start/Stop/Return/Abort |
| srv | `SetNavigationTarget` | Set/clear the navigation waypoint |
| action | `SetTargetDepth` | Long-running, cancellable depth change with feedback |

Conventions fixed once and used everywhere: ENU world frame; `z` up, so a submerged
vehicle has `z = -depth`; `depth` is also carried as its own field because it is the
primary control/display variable and, on a real vehicle, is measured directly by a
pressure sensor. `linear.z > 0` in the *control* Twist means **descend** (matching the
plant's internal sign); the Gazebo adapter flips it for Gazebo's z-up frame.

### 4.2 Telemetry and QoS

`vehicle_node` publishes three topics because telemetry, status and liveness have three
different reliability requirements that one QoS profile cannot satisfy.

| Topic | Rate | Reliability | Durability | History | Reasoning |
|---|---|---|---|---|---|
| `/auv/telemetry` | 20 Hz | BEST_EFFORT | VOLATILE | KEEP_LAST 5 | A stale sample is worthless once a fresher one exists ≤50 ms later. Best-effort avoids retransmission storms and back-pressure from a slow consumer (e.g. a GUI redraw), and mirrors what a lossy acoustic/serial link would force on real hardware. |
| `/auv/status` | 2 Hz | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST 1 | Status transitions (e.g. entering FAULT) must never be silently dropped, and a ground station that connects mid-mission must receive the *current* status immediately, not wait for the next change. TRANSIENT_LOCAL is exactly that latching behaviour. |
| `/auv/heartbeat` | 1 Hz | RELIABLE | VOLATILE | KEEP_LAST 1 | Only the latest matters, but heartbeat *loss* is the signal being watched, so the transport must not silently drop one and mask a real interruption. |

`VehicleState.status` echoes the status code so a telemetry-only consumer needs no second
subscription; `/auv/status` remains the authoritative, latched source for transitions.

**Ground truth vs observation.** `VehicleSimModel.state` holds noise-free internal state;
`step()` returns a separately perturbed copy (σ: depth 2 cm, x/y 3 cm, roll/pitch 0.01 rad).
Status logic (e.g. the battery fault threshold) reads ground truth, so it cannot flap on
its own injected noise. **Yaw is not perturbed in either backend** — relevant to §5.2.3.

**QoS pairing is a correctness issue, not a tuning detail.** A RELIABLE subscriber never
matches a BEST_EFFORT publisher; the connection silently fails. Consumers of
`/auv/telemetry` must therefore subscribe best-effort. This was violated once and caught
(§7, defect #11). A `COMMAND_QOS` profile is defined in `qos_profiles.py` but is **not
currently applied** — services and the action use rclpy defaults (reliable). It is left
as a documented placeholder rather than presented as in use.

### 4.3 Command interface

| Command | Mechanism | Why |
|---|---|---|
| Arm / Disarm | Service `SetArmed` | Instantaneous, needs a synchronous accept/reject *with a reason* (arming is refused in FAULT). A topic gives no acknowledgement. |
| Start / Stop / Return / Abort | Service `MissionCommand` (one service, `command` code) | Same accept/reject shape for all four; one parameterised service avoids four near-identical definitions. They are triggers, not durations: the multi-minute "returning" behaviour belongs to Mission Management, which can expose its own action when it exists. |
| Set Target Depth | Action `SetTargetDepth` | Takes seconds to tens of seconds, benefits from continuous feedback, and must be cancellable mid-dive. A service would either block for an unknown time or reply before the vehicle arrived. |

**Mission state machine** (`MissionStateMachine`, pure Python, 12 tests):

```
DISARMED ──arm──► ARMED ──START──► MISSION_ACTIVE ⇄(STOP / START)⇄ MISSION_PAUSED
                     ▲                    │                              │
                     │                    └──────RETURN / ABORT──────────┘
                     │                                   ▼
          FAULT ─(cleared)─► ARMED                    RETURNING
```

- Disarm **always wins** and cancels any mission (fail-safe direction is never blockable).
- `ABORT` is deliberately the most permissive command: allowed from any armed state,
  including `FAULT` and mid-return; the only refusal is `DISARMED` (nothing to abort).
- `FAULT` (battery < 15 %) clears to `ARMED`, **not** to the prior mission state, so a
  mission cannot silently resume on its own; the operator must re-issue `START`.
- `RETURN`/`ABORT` command a surface transit (target depth 0).

**Concurrency.** The depth action's execute callback blocks in a feedback loop, so
`vehicle_node` uses a `MultiThreadedExecutor` (4 threads) with a `ReentrantCallbackGroup`;
otherwise a dive would freeze telemetry and the heartbeat.

**Action semantics.** Goals with negative depth are rejected at acceptance; goals while
disarmed abort with a reason; default tolerance 0.15 m; feedback every 0.5 s; 60 s timeout
aborts a stuck dive; cancel is always accepted. All are parameters.

**Interaction with the autonomy override (known limitation).** While `auv_control` is
running it continuously overrides vehicle motion (§5.2). The action's internal target-depth
mechanism then has no effect, so a dive requested via the ground station's "Go to depth"
button will run until its 60 s timeout. This is a real, unresolved interaction (§9).

**Known simplification.** Entering `FAULT` does not auto-issue an abort; it blocks arming
and forces an explicit `START`. A real vehicle would auto-return on critical battery.

### 4.4 Ground station

PySide6 was chosen because it is what AUV/ROV competition teams typically build for a
custom ground station, is the same language as the ROS nodes (no rosbridge/web layer), and
suits a judge-facing demo. (PySide6 is **not** packaged for Ubuntu 24.04 — `apt-cache
policy python3-pyside6.qtwidgets` is empty there while PySide2 is available — so it is
installed with pip.)

**Threading model.** `rclpy` and Qt each want an event loop, so the app uses two threads:

- A background thread runs `while rclpy.ok(): executor.spin_once()`. **All rclpy calls
  happen only there** (subscriptions, service and action clients).
- The main thread runs Qt (`app.exec()`) and owns every widget.

*ROS → GUI* uses Qt signals emitted from the ROS thread. This is safe with no locking
because Qt decides queued-vs-direct delivery from the **receiver's** thread affinity, and
`MainWindow` lives on the main thread, so emissions are queued onto its event loop.

*GUI → ROS* deliberately does **not** use the reverse mechanism. Queued delivery requires
the *receiving* thread to run a Qt event loop, and the ROS thread runs a plain spin loop.
Button handlers instead put a tuple on a thread-safe `queue.Queue`; the ROS loop drains it
every ≈50 ms. This is simpler and more robust than forcing Qt to signal into a thread it was
not designed to signal.

`main_window.py` imports no `rclpy`, so the UI was constructed and rendered off-screen
(`QT_QPA_PLATFORM=offscreen`) in a ROS-free sandbox **[S]**, then launched for real **[R]**.

**Communication-failure indication.** `CommsMonitor` (pure Python, 7 tests) declares comms
lost from **time since the last heartbeat**, deliberately *not* from the content of the
last telemetry or status. A stream of stale-but-nominal-looking status is exactly the
failure a dedicated heartbeat exists to catch. Never having received one counts as lost
(not "unknown"). Timeout = 3 × heartbeat period (parameterised) so ordinary jitter does not
false-trigger. The GUI shows a colour-coded comms badge and logs transitions.

### 4.5 Simulation environment

**Design chosen: `VelocityControl`, not a thruster model.** The vehicle is a single rigid
cylinder (r = 0.15 m, L = 1.0 m). The adapter publishes a body-frame `Twist` on
`/model/auv/cmd_vel`; Gazebo's `VelocityControl` system tracks it; `OdometryPublisher`
returns ground-truth pose and twist; `Buoyancy` applies Archimedes' principle. This
sidesteps sign and gain uncertainty of raw thruster forces and keeps the interface a plain
`Twist`, which is what a control loop naturally produces. **[D]** for each plugin's name,
default topic and parameters, checked against the Gazebo 8.15.0 documentation.

**Neutral buoyancy by construction.** `mass = π r² L ρ = π(0.15)²(1.0)(1000) = 70.7 kg`,
against a world `uniform_fluid_density` of 1000 kg/m³, so with no command the hull neither
sinks nor rises. (Setting mass slightly higher would give the realistic slightly-negative
buoyancy of many AUVs; this was left neutral for clarity.)

**Bridge.** `ros_gz_bridge` config carries exactly four topics, each one-directional:
`cmd_vel` (ROS→Gazebo), `odometry`, `clock`, `camera` (Gazebo→ROS).

**`GazeboVehicleAdapter`** implements the same interface as `VehicleSimModel`
(`set_armed`, `set_target_depth`, `set_velocity_override`, `step`, `.state`), so
`vehicle_node` selects the backend with one parameter and every telemetry/command code path
is identical. It lives in `auv_vehicle` (not `auv_simulation`) which removes any circular
dependency. Two deliberate choices: **noise is added on top of Gazebo's ground truth**
(otherwise the estimator would have nothing to filter), and **battery remains a software
model** (Gazebo has no battery physics here), reusing `VehicleSimModel`'s drain formula so
it exists in one place.

**Vision additions.** A forward camera (640×480, 80° horizontal FOV, 15 Hz) on the nose
and a static orange target buoy (r = 0.4 m) at (5, 0, −2). Geometry was checked rather than
assumed **[S]**: vertical FOV = 2·atan(tan(40°)·480/640) = 64.4° (half-angle 32.2°); the buoy
centre lies 21.8° below the optical axis and its radius subtends a further ≈4.6°, so it is
fully in frame from spawn. At the original 3 m depth it would have been ≈1° inside the
frame edge, half cut off; it was moved (defect #12).

---

## 5. Autonomy Head

### 5.1 Navigation & Localization (`auv_navigation`)

| Requirement | Implementation |
|---|---|
| Position estimation | `StateEstimator`: filtered x, y, depth, yaw |
| Target representation | `SetNavigationTarget` service → stored target (x, y, depth) |
| Position/heading error | `navigation_math` → `distance_remaining`, `depth_error`, `heading_error` |
| Trajectory visualisation | `/auv/navigation/trajectory` (`nav_msgs/Path`, bounded to 2000 poses) |
| Sensor noise | Exponential filter; yaw filtered as (sin, cos) |

#### 5.1.1 Filter choice

A first-order exponential filter, not a Kalman filter. A Kalman filter needs a process
model; in this project the natural one is the simulator's own kinematics, which would mean
feeding the simulator's model back into the thing meant to check it — circular, and it
would not demonstrate what a filter is supposed to demonstrate. The exponential filter
does what "basic handling of sensor uncertainty" asks (smooths the injected noise, lags a
real step by a bounded and tunable amount) and is honest about being simple.

`α = 1 − exp(−Δt/τ)`; at 20 Hz with τ = 0.5 s, α ≈ 0.095. A large Δt drives α → 1, so a
long gap makes the filter trust the new measurement rather than drag out a stale estimate.

#### 5.1.2 Wrap-safe yaw

Averaging +179° and −179° linearly gives 0° (wrong; the true mean is ±180°). Yaw is filtered
as `sin` and `cos` separately and recombined with `atan2`. Every heading error goes through
`wrap_to_pi`, because naive subtraction breaks at ±π (desired +3.0 rad, current −3.0 rad are
≈0.28 rad apart on the circle, not ≈6.0). Tested at the boundary.

#### 5.1.3 Distance and depth are kept separate

`distance_remaining` is planar only. Folding depth into a 3-D distance would make the
"arrived horizontally" check and the depth loop interfere with each other. Depth is tracked
and controlled independently.

#### 5.1.4 Measured behaviour **[S]**

| Quantity | Predicted | Measured |
|---|---|---|
| Noise std reduction (3 cm input, 20 Hz, τ = 0.5 s) | ×0.224 = √(α/(2−α)) | 2.98 cm → **0.68 cm** (×0.227) |
| Steady-state lag while cruising at 0.4 m/s | vτ = 0.200 m | **0.190 m** |

This is the central trade-off, stated plainly: the filter cuts noise by ~4.4× but the
estimate trails true position by ≈0.19 m at cruise speed, **and that lag is inside the
control loop.** τ is a parameter (`filter_time_constant_s`).

#### 5.1.5 Underwater localization versus GPS

GPS needs line of sight to satellites; its radio signals are attenuated to nothing within
centimetres to a few metres of seawater, so a submerged AUV receives **no** GPS fix, only
briefly after surfacing. Real AUVs therefore:

- **dead-reckon** — integrate DVL-measured velocity and IMU/compass heading from the last
  known position (this filter is a heavily simplified stand-in for "estimate forward from the
  last state");
- measure **depth directly** with a pressure sensor (accurate, unlike x/y);
- correct accumulated drift with **acoustic positioning** (USBL/LBL ranging against a ship
  or seabed beacons) or by **surfacing for a GPS fix**.

The consequence is qualitative: dead-reckoning error grows without bound between
corrections, so an underwater estimator is fundamentally **bounding drift**, not merely
denoising an otherwise-accurate absolute fix as it would be for a GPS-equipped ground robot.
*In this simulation there is no drift* — the "sensor" is ground truth plus Gaussian noise —
so the filter demonstrates denoising only; drift, DVL and USBL modelling are not claimed.

### 5.2 Control System (`auv_control`)

#### 5.2.1 Structure

Two independent PID loops — depth (output = vertical velocity) and heading (output = yaw
rate) — reading `depth_error` and `heading_error` from `NavigationState`. Forward speed is a
cosine-scaled cruise (`cruise_speed · max(0, cos(heading_error))`, zero within the arrival
tolerance): it slows while badly misaligned so the vehicle turns toward the target rather
than overshooting past it. It is deliberately **not** a third PID: the requirement is depth
and heading, and speed exists only so the loop can be shown end to end.

Depth is controlled regardless of horizontal target. With no target set, both PIDs are reset
and zero is commanded (defect #13 explains the reset).

#### 5.2.2 Controller properties (`PIDController`, pure Python, 10 tests)

- **Anti-windup by clamped integration.** The integral stops accumulating while the output is
  saturated in the direction the error pushes. Without it, a large sustained error winds the
  integral far past what is needed and produces a long overshoot on unwinding. *Precisely
  what it guarantees:* the integral cannot grow without bound while saturated. It does **not**
  guarantee instant recovery — a stored integral still has to be worked off (§7, defect #7).
- **Derivative on measurement.** A setpoint step (frequent here — every new target) would
  otherwise cause a derivative kick from the setpoint's own discontinuity.
- **Angular mode.** For a wrapping measurement, the D-term must differentiate the *wrapped*
  change. See defect #8.
- **Non-positive `dt` is a no-op.**
- **Live tuning.** Gains are ROS parameters with a set-parameter callback:
  `ros2 param set /control_node depth_kp 3.0` applies on the next control step, and negative
  gains are rejected. This exists so the effect of each gain can be *shown*, which the
  assessment asks for.

#### 5.2.3 Tuning by simulation **[S]**

The plant is a velocity-commanded integrator: command m/s, depth accumulates. Tuning used
the *real* `StateEstimator` and *real* `PIDController` with sensor noise (depth σ = 2 cm; yaw
σ = 0.01 rad), the output clamps, and 20 Hz updates.

A 5 m depth step cannot discriminate gains: the 0.6 m/s clamp dominates (≥ 8.3 s whatever
the gains), and all candidate sets produced the same 7.5–8.0 s response. Gains were therefore
scored on three things that *do* differ: a small 0 → 0.5 m step, command jitter while holding,
and rejection of a constant −0.05 m/s drift (a stand-in for slight negative buoyancy).

| Depth gains (Kp, Ki, Kd) | Overshoot | Settle (5 %) | Hold jitter (command std) | Drift error |
|---|---|---|---|---|
| 40, 4, 15 *(initial guess)* | 11.5 % | never (39.8 s) | **0.471 m/s** | 0.015 m |
| 0.8, 0.05, 0.1 | 9.0 % | 8.5 s | 0.006 m/s | 0.002 m |
| 1.0, 0.05, 0.2 | 7.0 % | 6.6 s | 0.010 m/s | — |
| **1.5, 0.1, 0.3 (default)** | 8.7 % | **3.5 s** | 0.015 m/s | 0.003 m |
| 1.2, 0.08, 0.4 | 8.2 % | 7.6 s | 0.019 m/s | 0.003 m |
| 1.5, 0.1, 0.5 | 6.1 % | 5.7 s | 0.023 m/s | 0.003 m |
| 1.2, 0.1, 0.6 | 10.0 % | 11.6 s | 0.027 m/s | 0.003 m |

The initial gains saturated the clamp at 1.5 cm of error, i.e. near bang-bang. An honest
reading of the evidence: the *depth trace* looked only modestly noisier, because the
integrating plant smooths the chatter. The real cost was **actuator chatter** — a physical
thruster would have been driven almost full-scale back and forth while "holding" depth.

Ki's disturbance rejection, isolated: with Ki = 0 the steady error under drift was 0.063 m;
with Ki = 0.05 it was 0.002 m.

Heading defaults (Kp 1.2, Ki 0.05, Kd 0.4) gave 1.6 % overshoot and ≈3.3 s settle for a 90°
step. Because neither backend perturbs yaw, the σ = 0.01 rad used in tuning is a
**conservative** assumption, not a measurement of the real stream.

**Effect of each parameter (observed above):**

| Parameter | Raising it | Too high |
|---|---|---|
| Kp | Faster response, smaller error | Overshoot, and sensor noise amplified into command jitter (the 0.471 m/s row) |
| Ki | Removes steady-state error under constant disturbance | Overshoot, slow oscillation; anti-windup bounds the worst of it |
| Kd | Damps the approach | Amplifies measurement noise (D acts on noisy data) |
| Output clamp | Protects the actuator | Slows large steps (it alone sets the ≈8 s for 5 m) |

#### 5.2.4 Limits of this control evidence

Tuned against an idealised integrator plant with a modelled estimator, **not** against
Gazebo. `VelocityControl` tracks velocity essentially ideally, so there is no thruster
lag, drag or added mass: real-plant behaviour will differ and gains should be re-checked
once run in Gazebo. Loops are independent; no coupling or feed-forward.

### 5.3 Computer Vision (`auv_perception`)

**Pipeline:** camera frame → HSV threshold → open/close morphology (5 px kernel) → external
contours → largest contour → centroid, minimum enclosing circle, fill ratio.

**Output (`TargetDetection`):** `found`; raw pixel coordinates and image size; **normalised**
coordinates in [−1, 1] (0,0 at centre, +x right, +y up), which are resolution-independent and
what a downstream visual-servo controller or Mission Management should consume;
`apparent_radius_px` (a rough closeness proxy); `confidence`. Published even when
`found = false`, so consumers can see "lost" rather than silence.

**Why colour thresholding, not ArUco or YOLO.** The brief permits a coloured object and
"OpenCV, ArUco, YOLO, or other". HSV thresholding is fully explainable (each parameter has
one visible effect — important for a criterion about justifying decisions), needs no dataset,
model or GPU (CPU at the camera's 15 Hz with wide margin), and a coloured sphere needs no
texture asset in Gazebo whereas an ArUco marker does.

**Trade-off, stated honestly.** Colour thresholding is sensitive to lighting and water
colour. Measured **[S]**: the buoy's material (HSV ≈ 10, 255, 255) stays inside the default
range (5,150,120)–(25,255,255) down to ≈50 % of full brightness and falls out at ≈40 %.
Real underwater cameras lose red first with depth, which is the main reason a competition
system would move to a learned detector. A pure-red target would also need two hue ranges
(red straddles hue 0/179); this class does not handle that, and its docstring says so (it
originally claimed to — defect #14).

**Confidence** is the fraction of the blob's own enclosing circle actually filled by matching
pixels (≈1 for a clean disc, lower for ragged matches). It is *not* a calibrated probability.

**Robustness handled and tested:** specks below `min_area_px = 60`, wrong-colour blobs,
multiple blobs (largest wins), no target, unsupported image encodings (rejected loudly).

**No `cv_bridge`.** The apt `cv_bridge` is built against the system NumPy and fails on import
if another NumPy (e.g. a pip 2.x) precedes it on `PYTHONPATH` — which this workspace's
environment trick can cause. `image_conversion.py` converts `rgb8/bgr8/mono8` in plain numpy,
handling row padding via `step`, RGB→BGR swap and mono expansion (6 tests).

---

## 6. Testing strategy and results

### 6.1 Results **[U]**

| Package | Tests | Covers |
|---|---|---|
| `auv_vehicle` | 22 | Plant model (10: depth convergence, disarmed hold, battery, override priority, stale-override fallback) · mission FSM (12: every transition, abort permissiveness, fault clearing) |
| `auv_ground_station` | 7 | Comms monitor: never-received, timeout boundaries, recovery |
| `auv_navigation` | 14 | Filter (noise, step tracking, yaw wrap, `dt ≤ 0`, reset) · navigation math (wrap, bearing, heading error at the boundary) |
| `auv_control` | 10 | P/PI behaviour on a first-order plant, clamping, anti-windup, derivative damping, angular wrap, `dt ≤ 0`, reset |
| `auv_perception` | 15 | Detector on synthetic frames (location, sign convention, speck, wrong colour, multi-blob, confidence, annotate purity) · image conversion |
| **Total** | **68** | |

### 6.2 What the tests found

Tests earned their keep: the PID anti-windup test failed on first run (defect #7, a wrong
expectation in the test, not the code) and forced the precise statement of what anti-windup
guarantees. Simulation (not tests) found defects #8 and #9.

### 6.3 What the tests cannot see

Every test exercises a pure core. **None** exercises ROS wiring: topic names, QoS
compatibility, message field mapping, executor behaviour, launch composition, the Gazebo
bridge, or the GUI beyond off-screen construction. This is the direct cost of principle P1 and
it produced a real defect (#11). A `launch_testing` smoke test — bring the graph up, publish a
telemetry message, assert a `NavigationState` appears — would close most of the gap and is
the highest-value test still missing.

---

## 7. Defects found and fixed during development

| # | Defect | Found by | Resolution / lesson |
|---|---|---|---|
| 1 | `colcon build` ran under system Python regardless of the active venv; `ModuleNotFoundError: em` | Build error | `colcon` is an apt script with a hard-coded `/usr/bin/python3` shebang; activating a venv cannot change it. Jazzy needs `empy==3.3.4`. Extra pip packages reach system Python via `PYTHONPATH` to the venv's site-packages. |
| 2 | Duplicate package names (`auv_interfaces`, `auv_vehicle`) | `colcon` error | Archive was extracted inside `src/`, creating nested copies. Extract from the workspace root. |
| 3 | `setup.cfg` dash-separated options | Deprecation warning | Use `script_dir` / `install_scripts`. |
| 4 | Qt "xcb" plugin failed to load | Runtime error | `libxcb-cursor0` (a system library pip cannot install) is required by Qt ≥ 6.5. |
| 5 | PySide6 absent from apt on Noble | `apt-cache policy` | Verified rather than assumed; pip is the only route on this release. |
| 6 | SDF/XML comments containing `--` are not well-formed | XML parse (three occurrences) | Reworded. XML forbids a double hyphen inside a comment. |
| 7 | Anti-windup test asserted near-instant recovery from a barely-reversed error against a huge stored integral | Failing test | The test was wrong, not the code: clamped integration bounds growth but does not unwind instantly. Test rewritten to assert the real invariant. |
| 8 | **Yaw wrap spike:** D-term differentiated raw yaw; crossing ±π produced a ≈2π "jump" | Closed-loop simulation | `angular` mode wraps the measurement delta. Measured on a ≈0.18 rad move across the boundary: peak command **0.50 rad/s (saturated) without the fix, 0.22 rad/s with it.** |
| 9 | **Depth gains badly tuned** (Kp 40): 0.47 m/s command jitter while holding | Simulation scored on jitter, not just step response | Retuned to 1.5 / 0.1 / 0.3; table in §5.2.3. A large step response *cannot* show this — the rate clamp hides it. |
| 10 | `cv_bridge` / NumPy ABI hazard | Reasoning about the `PYTHONPATH` trick | Avoided `cv_bridge` entirely; tested pure-numpy conversion. |
| 11 | **QoS mismatch:** `navigation_node` subscribed to best-effort `/auv/telemetry` with the reliable default, so it would have received *no data* | Audit of every pub/sub pair while preparing this document | Now uses `qos_profile_sensor_data` (BEST_EFFORT, VOLATILE, KEEP_LAST 5). **Shipped in an earlier archive; corrected in `…_v2`.** No unit test could catch it (§6.3). |
| 12 | Target buoy only ≈1° inside the camera's vertical FOV edge at 3 m | Geometry check | Moved to 2 m; world comment records the numbers. |
| 13 | After a target was cleared, a leftover depth integral would keep commanding vertical velocity | Review | Depth PID now reset with the heading PID when no target. |
| 14 | Detector docstring claimed a dual hue range that was not implemented | Review | Docstring corrected; the limitation is stated instead. |
| 15 | README stated the wrap test moved 0.08 rad; it was ≈0.18 rad | Recalculation | Corrected. Recorded because a wrong number in documentation is a defect like any other. |

---

## 8. Assumptions

1. Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.x; `ros_gz_sim` and `ros_gz_bridge` installed.
2. Colcon nodes run under **system** Python; extra pip dependencies are made visible through `PYTHONPATH`. `python3-opencv` is installed via apt for the vision node.
3. Simulation time and wall time are treated as effectively the same for control; `/clock` is bridged but the autonomy nodes use the node clock.
4. The "sensor" is ground truth plus Gaussian noise (σ: depth 2 cm, x/y 3 cm, roll/pitch 0.01 rad); yaw is clean.
5. `VelocityControl` tracks commanded velocity ideally (no thruster lag, drag or added mass).
6. The target is a single saturated orange sphere under nominal Gazebo lighting.
7. Battery is a linear software model (6 %/h armed, 0.4 %/h idle), with a 15 % fault threshold.
8. One operator, one vehicle, one ground station; no security or authentication on the command interface.

---

## 9. Limitations, known gaps and unverified items

**Not verified (must be run before these are relied on)**
- All autonomy ROS nodes (`navigation_node`, `control_node`, `vision_node`) and `auv_bringup`.
- The Gazebo camera: that `<topic>camera</topic>` publishes exactly `/camera`, that it faces along the hull's +x, and that the buoy appears in frame. If `/camera` is empty: `gz topic -l | grep -i camera`.
- The Gazebo world/model/bridge end to end, and the ground-station command paths against it.
- RViz `Path` display and `rqt_image_view` overlay.

**Known design limitations**
- **Control ↔ command-interface conflict** (§4.3): with `auv_control` running, the ground station's "Go to depth" cannot move the vehicle and times out at 60 s. Options: make the button set the navigation target, or have control publish nothing when no target exists (which restores the old spinning search pattern).
- `COMMAND_QOS` is defined but unused (§4.2).
- `FAULT` does not auto-return (§4.3).
- Control tuned on an idealised plant; expect re-tuning in Gazebo (§5.2.4).
- Estimator lag ≈0.19 m at cruise sits inside the control loop (§5.1.4).
- No drift, DVL or acoustic positioning, so localization is denoising only (§5.1.5).
- HSV detection is lighting-sensitive and single-target; no learned detector (§5.3).
- Roll/pitch are not estimated or controlled; navigation is planar plus depth.
- No obstacle handling anywhere yet (there are no obstacles in the world).

**Process limitation**
- The development sandbox has no ROS and no Gazebo. That is why the pure-core structure was adopted, and why "unit-tested" must never be read as "integration-tested".

---

## 10. Remaining work

| Item | Notes |
|---|---|
| **Motion Planning** (Autonomy 3) | Waypoint/A*-style planner with obstacles, velocity and turn limits, path length, energy; justify the choice. Will consume the navigation estimate and produce waypoints for `SetNavigationTarget`. |
| **Autonomous Mission Management** (Autonomy 5) | State machine: start → search → detect → estimate target position → plan → navigate → reached → complete. This is where `TargetDetection` (normalised image coordinates + apparent radius) becomes a navigation target, and where the resolved "who owns depth" question (§9) belongs. |
| Run-through of §1.3 **[?]** items | Highest priority; do before building further. Appendix C. |
| ROS-level smoke test | `launch_testing`; would have caught defect #11. |
| Ground-station panels | Navigation state and vision detection are not yet displayed. |
| Architecture diagram update | Workspace README diagram predates the autonomy packages; §2.1 here is current. |
| Submission packaging | Repository README, 3–5 min video, final technical documents split per role. |

---

## 11. Mapping to the assessment criteria

### Software Head

| Criterion | Evidence | Status |
|---|---|---|
| ROS 2 & middleware (nodes, topics, services, actions, QoS) | §4.2–4.3: three QoS profiles with reasoning; services vs action justified; multithreaded executor | Strong; QoS-pairing lesson (#11) shows it matters |
| Software architecture | §2, §3: layered, single state owner, named seams, launch-only bringup package | Strong |
| Communication (reliability, error handling) | Heartbeat-based comms loss; reliable latched status; service reject-with-reason; action timeout/cancel | Strong on design; end-to-end failure injection **[?]** |
| Vehicle software | FSM (12 tests), telemetry, commands | Strong |
| Ground station | Monitoring + control + failure indication; threading rationale | Implemented **[R]** |
| Simulation | Gazebo integration, `VelocityControl`, buoyancy, camera, bridge | Implemented; **[?]** not yet run end to end |
| Code quality | Pure-core/thin-shell, 68 tests, lint stubs, deprecation clean-up | Good |
| Documentation | Per-package READMEs, this document | Good; repo-level README and diagram refresh outstanding |
| Problem solving (failures, edge cases) | §7: 15 defects, causes and lessons; ABORT permissiveness; fault-clearing rule; stale-override fallback | Strong |

### Autonomy Head (completed portion)

| Criterion | Evidence | Status |
|---|---|---|
| Navigation | §5.1; measured noise reduction and lag; GPS-vs-underwater explanation | Implemented **[U][S]** |
| Computer vision | §5.3; detector tests; HSV robustness measured | Implemented **[U][S]**; live Gazebo frames **[?]** |
| Motion planning | — | **Not started** |
| Controls | §5.2; tuning table; parameter effects; live-tunable | Implemented **[U][S]**; Gazebo response **[?]** |
| Autonomy (end to end) | — | **Not started** (needs mission management) |
| ROS 2 integration | Clean topic contracts; QoS audit | Nodes unrun **[?]** |
| System integration | Closed loop wired (§2.3) | Unrun **[?]** |
| Robustness (noise, failures, edge cases) | Noise filtering, wrap handling, stale-override fallback, speck/multi-blob rejection | Good on logic |
| Technical understanding | Tradeoffs stated at every decision; limitations named | Strong |
| Documentation | This document + READMEs | Good |

---

## Appendix A — Interface reference

**Topics**

| Topic | Type | Rate | QoS | Publisher → Subscriber |
|---|---|---|---|---|
| `/auv/telemetry` | `VehicleState` | 20 Hz | best-effort, volatile, 5 | vehicle → GUI, navigation |
| `/auv/status` | `SystemStatus` | 2 Hz | reliable, transient-local, 1 | vehicle → GUI |
| `/auv/heartbeat` | `Heartbeat` | 1 Hz | reliable, volatile, 1 | vehicle → GUI |
| `/auv/navigation/state` | `NavigationState` | = telemetry rate | default | navigation → control |
| `/auv/navigation/trajectory` | `nav_msgs/Path` | = telemetry rate | default | navigation → RViz |
| `/auv/control/cmd_vel` | `geometry_msgs/Twist` | = navigation rate | default | control → vehicle |
| `/auv/vision/target_detection` | `TargetDetection` | ≤ 15 Hz | default | vision → (consumers TBD) |
| `/auv/vision/debug_image` | `sensor_msgs/Image` | ≤ 15 Hz | default, depth 1 | vision → viewers |
| `/camera` | `sensor_msgs/Image` | 15 Hz | bridge default | bridge → vision |
| `/model/auv/cmd_vel` | `Twist` | telemetry rate | default | adapter → bridge |
| `/model/auv/odometry` | `nav_msgs/Odometry` | 20 Hz | default | bridge → adapter |

**Services / action**

| Name | Type |
|---|---|
| `/auv/arm` | `SetArmed` |
| `/auv/mission_command` | `MissionCommand` (0 START, 1 STOP, 2 RETURN, 3 ABORT) |
| `/auv/navigation/set_target` | `SetNavigationTarget` |
| `/auv/set_target_depth` | action `SetTargetDepth` |

**Key parameters**

| Node | Parameter | Default |
|---|---|---|
| `vehicle_node` | `telemetry_rate_hz` / `status_rate_hz` / `heartbeat_rate_hz` | 20 / 2 / 1 |
| | `use_gazebo`, `initial_armed`, `sim_seed` | false, false, −1 |
| | `depth_tolerance_m` / `depth_action_timeout_s` / `depth_feedback_period_s` | 0.15 / 60 / 0.5 |
| `ground_station` | `vehicle_heartbeat_rate_hz`, `heartbeat_timeout_multiplier` | 1.0, 3.0 |
| `navigation_node` | `filter_time_constant_s`, `trajectory_max_poses` | 0.5, 2000 |
| `control_node` | `depth_kp/ki/kd` | 1.5 / 0.1 / 0.3 |
| | `heading_kp/ki/kd` | 1.2 / 0.05 / 0.4 |
| | `max_vertical_speed_mps`, `max_yaw_rate_radps` | 0.6, 0.5 |
| | `cruise_speed_mps`, `arrival_tolerance_m` | 0.4, 0.5 |
| `vision_node` | HSV lower / upper | (5,150,120) / (25,255,255) |
| | `min_area_px`, `publish_debug_image` | 60, true |

## Appendix B — Package layout

```
src/
├── auv_interfaces/        msg/ srv/ action/
├── auv_vehicle/           vehicle_node · gazebo_adapter · vehicle_sim_model ·
│                          mission_state_machine · qos_profiles · telemetry_publisher
├── auv_ground_station/    app · main_window · ros_bridge · comms_monitor
├── auv_simulation/        worlds/ models/auv/ config/auv_bridge.yaml launch/
├── auv_navigation/        navigation_node · state_estimator · navigation_math
├── auv_control/           control_node · pid_controller
├── auv_perception/        vision_node · color_detector · image_conversion
└── auv_bringup/           launch/autonomy.launch.py
```

## Appendix C — Verification checklist (close the `[?]` items)

```bash
# 0. Setup
sudo apt install python3-opencv libxcb-cursor0
cd ~/AUV_int_task-2026
tar -xzf ~/Downloads/auv_int_task-2026_autonomy_update_v2.tar.gz
rm -rf build install log && colcon build && source install/setup.bash

# A. Navigation + control against the built-in plant (no Gazebo)
ros2 launch auv_bringup autonomy.launch.py use_gazebo:=false initial_armed:=true ground_station:=false
#   (second terminal, sourced)
ros2 topic hz /auv/navigation/state          # expect ≈ 20 Hz  (would be silent if defect #11 recurred)
ros2 service call /auv/navigation/set_target auv_interfaces/srv/SetNavigationTarget \
  "{x: 5.0, y: 2.0, depth: 2.0, clear: false}"
ros2 topic echo /auv/navigation/state        # distance_remaining should fall
ros2 param set /control_node depth_kp 6.0    # depth should visibly get twitchier

# B. Gazebo camera and detection
ros2 launch auv_bringup autonomy.launch.py initial_armed:=true
gz topic -l | grep -i camera
ros2 topic hz /camera
ros2 run rqt_image_view rqt_image_view /auv/vision/debug_image     # green circle on the buoy
ros2 topic echo /auv/vision/target_detection
```
