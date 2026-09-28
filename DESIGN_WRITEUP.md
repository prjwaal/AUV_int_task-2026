Technical Design & Architecture Document — AUV Software & Autonomy StackTarget Environment: Ubuntu 24.04 LTS (Noble) · ROS 2 Jazzy · Gazebo Harmonic (8.15.0)Workspace Architecture: 8-Package Modular colcon Workspace (auv_bringup, auv_control, auv_ground_station, auv_interfaces, auv_navigation, auv_perception, auv_simulation, auv_vehicle)1. Unified System Architecture (Sense $\rightarrow$ Plan $\rightarrow$ Act)The AUV software stack is architected around a 4-Tier Layered Design combined with the Sense–Plan–Act autonomy paradigm and the Humble Object Pattern (isolating pure mathematical and state-machine logic from rclpy middleware so all core algorithms can be unit-tested deterministically without a running simulator).A. System Architecture DiagramPlaintext+========================================================================================+
|                     SUPERVISORY & BRINGUP LAYER (auv_ground_station / auv_bringup)     |
|                                                                                        |
|  +------------------------------+              +------------------------------------+  |
|  | Ground Station GUI (app.py,  |<------------>| ROS 2 Bridge & Comms Watchdog      |  |
|  | main_window.py)              |              | (ros_bridge.py, comms_monitor.py)  |  |
|  +------------------------------+              +-----------------+------------------+  |
+==================================================================|=====================+
                                                                   |
        +----------------------------------------------------------+
        | Services: /auv/arm, /auv/mission_command, /auv/set_navigation_target
        | Actions:  /auv/set_target_depth
        | Topics:   /auv/telemetry (20Hz), /auv/status (2Hz), /auv/heartbeat (1Hz)
        v
+========================================================================================+
|                         AUTONOMY PIPELINE: SENSE -> PLAN -> ACT                        |
|                                                                                        |
|  +---------------------------+   +--------------------------+   +-------------------+  |
|  | 1. SENSE: PERCEPTION      |   | 2. SENSE: NAVIGATION     |   | 3. PLAN & MISSION |  |
|  | (auv_perception)          |   | (auv_navigation)         |   | MANAGEMENT        |  |
|  | - vision_node.py          |   | - navigation_node.py     |   | - Target Search   |  |
|  | - color_detector.py       |   | - state_estimator.py     |   | - 3D Goal Update  |  |
|  | - image_conversion.py     |   | - navigation_math.py     |   | - Obstacle-Aware  |  |
|  |                           |   |                          |   |   A* Path Planner |  |
|  | In:  /camera              |   | In:  /auv/telemetry      |   +---------+---------+  |
|  | Out: /auv/vision/detection|   | Out: NavigationState.msg |             |            |
|  +-------------+-------------+   +------------+-------------+             |            |
|                |                              |                           |            |
|                +--------------+---------------+---------------------------+            |
|                               |                                                        |
|                               v                                                        |
|  +----------------------------------------------------------------------------------+  |
|  | 4. ACT: CLOSED-LOOP CONTROL (auv_control)                                        |  |
|  | - control_node.py & pid_controller.py                                            |  |
|  | - Multi-Axis PID (Depth PID + Heading/Yaw PID + Alignment-Scaled Surge Control)  |  |
|  +----------------------------------------+-----------------------------------------+  |
+===========================================|============================================+
                                            | Velocity / Depth Commands
                                            v
+========================================================================================+
|                         ONBOARD VEHICLE EXECUTIVE (auv_vehicle)                        |
|                                                                                        |
|  +----------------------------------------+-----------------------------------------+  |
|  | vehicle_node.py  <--->  mission_state_machine.py (DISARMED|ARMED|ACTIVE|RETURN)  |  |
|  +----------------------------------------+-----------------------------------------+  |
|                                           |                                            |
|                     +---------------------+---------------------+                      |
|                     v                                           v                      |
|        +-------------------------+                 +-------------------------+         |
|        | VehicleSimModel         |                 | GazeboVehicleAdapter    |         |
|        | (vehicle_sim_model.py)  |                 | (gazebo_adapter.py)     |         |
|        +-------------------------+                 +------------+------------+         |
+=================================================================|======================+
                                                                  |
                                     /model/auv/cmd_vel (Twist)   |   /model/auv/odometry
                                     /camera (Image)              v   /clock
+========================================================================================+
|                         PHYSICS & SENSOR SIMULATION (auv_simulation)                   |
|                                                                                        |
|  +----------------------------------------------------------------------------------+  |
|  | ros_gz_bridge (auv_bridge.yaml) <---> Gazebo Harmonic (auv_world.sdf, model.sdf) |  |
|  +----------------------------------------------------------------------------------+  |
+========================================================================================+
B. Mermaid Flowchart (Renders Automatically in GitHub README.md)Code snippetflowchart TB
    subgraph SIM["1. Simulation Layer (auv_simulation)"]
        GZ["Gazebo Harmonic (auv_world.sdf / model.sdf)<br/>Buoyancy + Odometry + Forward Camera"]
        BR["ros_gz_bridge (auv_bridge.yaml)"]
        GZ <--> BR
    end

    subgraph VEH["2. Vehicle Layer (auv_vehicle)"]
        VNODE["vehicle_node.py"]
        FSM["mission_state_machine.py"]
        ADAPT["gazebo_adapter.py / vehicle_sim_model.py"]
        VNODE <--> FSM
        VNODE <--> ADAPT
    end

    subgraph AUT["3. Autonomy Layer (Sense -> Plan -> Act)"]
        PERC["auv_perception (vision_node.py)<br/>color_detector.py / image_conversion.py"]
        NAV["auv_navigation (navigation_node.py)<br/>state_estimator.py / navigation_math.py"]
        PLAN["Motion Planner & Mission Executive<br/>Turn-Penalized A* + 8-Phase Mission FSM"]
        CTRL["auv_control (control_node.py)<br/>pid_controller.py (Depth & Heading PID)"]
    end

    subgraph GS["4. Supervisory Layer (auv_ground_station)"]
        GUI["Operator GUI (app.py / main_window.py)"]
        MON["Watchdog (comms_monitor.py)"]
        GUI <--> MON
    end

    BR -->|"/model/auv/odometry"| ADAPT
    BR -->|"/camera"| PERC
    VNODE -->|"/auv/telemetry (BEST_EFFORT)"| NAV
    VNODE -->|"/auv/telemetry, /auv/status, /auv/heartbeat"| GUI
    PERC -->|"/auv/vision/detection (TargetDetection.msg)"| PLAN
    NAV -->|"/auv/navigation/state (NavigationState.msg)"| CTRL
    PLAN -->|"/auv/set_navigation_target"| NAV
    CTRL -->|"/model/auv/cmd_vel (Twist)"| BR
    GUI -->|"/auv/arm, /auv/mission_command, /auv/set_target_depth"| VNODE
2. Navigation & Localization (auv_navigation)A. Why Underwater Localization Differs from GPS NavigationConventional surface and aerial robots rely on Global Navigation Satellite Systems (GNSS/GPS) operating in the L-band radio frequency spectrum ($1.1\text{--}1.6\text{ GHz}$). Underwater, GPS is unavailable because water is an electrically conductive, polar medium that rapidly attenuates electromagnetic waves. The skin depth $\delta$ of a radio wave in conductive water is given by:$$\delta = \sqrt{\frac{2}{\omega \mu \sigma}}$$At $1.5\text{ GHz}$, radio waves are absorbed within a few centimeters of the surface. Consequently, an AUV diving below the surface cannot receive continuous satellite position fixes and must rely on Dead Reckoning and Multi-Sensor Fusion:Hydrostatic Pressure Sensor (Depth $z$): Directly measures water column pressure ($P = \rho g d$), providing bounded, drift-free vertical depth estimates.Inertial Measurement Unit (IMU): Tri-axis gyroscopes and accelerometers estimate roll, pitch, and heading ($\phi, \theta, \psi$), though integrating linear acceleration alone leads to quadratic position drift ($\epsilon_p \propto t^2$).Doppler Velocity Log (DVL): Emits acoustic beams toward the seafloor to measure bottom-lock 3D velocity $(\dot{x}, \dot{y}, \dot{z})$, reducing dead-reckoning drift to linear growth over time.Acoustic & Visual Bounding (USBL / LBL / Vision): Long-term drift is corrected using Ultra-Short Baseline (USBL) acoustic transponders, periodic surfacing for GPS, or visual localization against known underwater landmarks/targets.B. State Estimation & Sensor Noise Handling (state_estimator.py)Because /auv/telemetry contains zero-mean Gaussian measurement noise on position, depth, and orientation, feeding raw telemetry directly into a derivative controller causes severe thruster chatter. state_estimator.py implements a First-Order Low-Pass Exponential State Filter (with configurable time constant $\tau = 0.5\text{ s}$):$$\alpha = \frac{\Delta t}{\tau + \Delta t}, \quad \hat{\mathbf{x}}_k = (1 - \alpha)\hat{\mathbf{x}}_{k-1} + \alpha \mathbf{z}_k$$For heading ($\psi$), filtering is performed on the unit circle using wrapped angular residuals so transitions across $\pm \pi$ never cause averaging spikes:$$\hat{\psi}_k = \operatorname{wrap\_pi}\!\left(\hat{\psi}_{k-1} + \alpha \cdot \operatorname{wrap\_pi}(\psi_{\text{meas}, k} - \hat{\psi}_{k-1})\right)$$C. Target Representation & Error Formulation (navigation_math.py)The active 3D target is represented in world coordinates as $\mathbf{p}_{\text{target}} = (x_t, y_t, d_t)$ and can be updated dynamically via /auv/set_navigation_target (SetNavigationTarget.srv). Given the filtered vehicle state $(\hat{x}, \hat{y}, \hat{d}, \hat{\psi})$, navigation_math.py computes and publishes NavigationState.msg:Position Error Vector:$$e_x = x_t - \hat{x}, \quad e_y = y_t - \hat{y}, \quad e_{\text{depth}} = d_t - \hat{d}$$Planar Euclidean Distance Error:$$d_{\text{planar}} = \sqrt{e_x^2 + e_y^2}, \quad d_{\text{3D}} = \sqrt{e_x^2 + e_y^2 + e_{\text{depth}}^2}$$Target Bearing & Wrapped Heading Error:$$\psi_{\text{target}} = \operatorname{atan2}(e_y, e_x)$$$$e_{\psi} = \operatorname{atan2}\!\left(\sin(\psi_{\text{target}} - \hat{\psi}),\; \cos(\psi_{\text{target}} - \hat{\psi})\right) \in [-\pi, \pi]$$3. Computer Vision Pipeline (auv_perception)A. Pipeline ArchitectureThe perception stack (vision_node.py, color_detector.py, image_conversion.py) processes live frames from the AUV's forward-facing camera (/camera bridged from Gazebo via ros_gz_bridge):Image Acquisition & Decoding (image_conversion.py): Converts incoming sensor_msgs/msg/Image payloads (rgb8 / bgr8) into contiguous NumPy arrays without external C++ bridge fragility.HSV Color-Space Segmentation (color_detector.py): Converts frames into HSV (Hue, Saturation, Value) space. Unlike RGB, HSV separates chromaticity ($H$) from illumination intensity ($V$), making detection resilient to underwater light absorption and depth-dependent dimming.Morphological Filtering & Connected-Component Moment Analysis: Applies thresholding and computes spatial image moments $(M_{00}, M_{10}, M_{01})$ to extract the target's pixel centroid $(u, v)$, normalized image offsets, and bounding area:$$u = \frac{M_{10}}{M_{00}}, \quad v = \frac{M_{01}}{M_{00}}, \quad \tilde{u} = \frac{u - W/2}{W/2} \in [-1, 1], \quad \tilde{v} = \frac{v - H/2}{H/2} \in [-1, 1]$$ROS 2 Output (TargetDetection.msg): Publishes /auv/vision/detection containing detected (bool), pixel centroid $(u, v)$, normalized bearing/elevation offsets, and bounding box dimensions, alongside an annotated debug video stream (/auv/vision/debug_image) with crosshair overlays.4. Motion PlanningA. Selected Approach: Turn-Penalized 2D A* with Line-of-Sight Pruning + Depth ProfilingBecause underwater environments feature sparse 3D obstacles and AUVs operate with decoupled pitch/heave and planar surge/yaw dynamics, the motion planner couples vertical depth profiling with 2D Turn-Penalized A* search and Line-of-Sight (Ray-Cast) waypoint smoothing.B. Engineering Justification vs. Alternative PlannersWhy A over Dijkstra:* Dijkstra expands nodes uniformly in all directions ($O(V \log V)$), wasting computation on regions away from the goal. A* uses an admissible octile/Euclidean heuristic $h(n) = \Vert{}\mathbf{p}_n - \mathbf{p}_{\text{goal}}\Vert{}_2$ to focus search directly toward the target while guaranteeing global optimality on the grid.Why A + Smoothing over RRT/RRT*:* Sampling-based planners like RRT produce stochastic, jagged paths that force torpedo-hull AUVs to execute frequent yaw oscillations. Deterministic A* combined with Line-of-Sight pruning collapses collinear grid steps into a minimal set of smooth, straight-line segments.C. Handling the 5 Core Planning ConstraintsPlanning ConstraintMathematical & Algorithmic Implementation1. ObstaclesObstacles at $(x_o, y_o, r_o)$ are inflated by the vehicle radius $r_{\text{auv}}$ plus a safety margin $d_{\text{safe}}$ ($r_{\text{inflated}} = r_o + r_{\text{auv}} + d_{\text{safe}}$). Grid cells inside $r_{\text{inflated}}$ are marked impassable, while cells within a buffer band incur a repulsive potential cost $J_{\text{prox}}$.2. Turning LimitationsUnderactuated AUVs cannot pivot sharply at high surge speeds. The A* transition cost penalizes heading changes between consecutive edges: $J_{\text{turn}} = w_\psi \vert{}\operatorname{wrap\_pi}(\psi_{k} - \psi_{k-1})\vert{}$, producing wide, dynamically feasible turns.3. Path LengthStandard 8-connected grid paths suffer from "staircase" artifacts. A post-processing Line-of-Sight Ray-Cast Pruner checks direct collision-free visibility between non-adjacent nodes and removes redundant intermediate waypoints, minimizing total Euclidean path length.4. Vehicle VelocitySurge velocity is modulated by heading alignment: $v_x = \operatorname{clip}(k_v d_{\text{planar}}, 0, v_{\max}) \cdot \max(0, \cos e_\psi)$. When facing away from the waypoint ($\vert{}e_\psi\vert{} > 90^\circ$), forward thrust drops to zero so the AUV rotates onto course before accelerating.5. Energy EfficiencyHydrodynamic drag scales quadratically with velocity ($F_{\text{drag}} = \frac{1}{2}\rho C_d A v^2$, power $P \propto v^3$), and yaw thrusters draw high peak current during stop-and-turn maneuvers. Minimizing path length, pruning unnecessary waypoints, and penalizing sharp heading changes directly minimizes total propulsion energy $E = \int (c_v v^2 + c_\omega \omega^2)\,dt$.5. Control System (auv_control)A. Multi-Axis PID Control Architecture (pid_controller.py & control_node.py)control_node.py subscribes to /auv/navigation/state (NavigationState.msg) and runs independent discrete-time PID controllers for Depth ($z$) and Heading ($\psi$) alongside a proportional, heading-gated Surge ($x$) controller:$$u(t) = \operatorname{clip}\!\left(K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{e(t) - e(t - \Delta t)}{\Delta t},\; -u_{\max},\; u_{\max}\right)$$To prevent integral windup during large initial depth dives or $180^\circ$ heading reversals, the accumulated integral term $I_k = I_{k-1} + e_k \Delta t$ is clamped within $[-I_{\max}, I_{\max}]$, and the controller state is reset (reset()) whenever the vehicle is disarmed or inactive.B. Physical Effect of PID Parameters on AUV DynamicsGainPhysical Meaning in Underwater ControlEffect if Too LowEffect if Too High$K_p$ (Proportional)Acts as a virtual spring pulling the AUV toward the target depth or heading proportionally to current error $e(t)$.Slow, sluggish response; fails to overcome hydrodynamic drag or buoyancy offsets.Overshoots target depth/heading and causes sustained oscillations.$K_i$ (Integral)Eliminates steady-state error caused by constant disturbances (e.g., slight positive/negative buoyancy or steady underwater currents).AUV settles with a persistent offset above or below the target depth.Integral windup: Accumulates excess error during transit, causing severe overshoot and slow recovery.$K_d$ (Derivative)Acts as a virtual damper opposing the rate of change of error $\dot{e}(t)$, braking the AUV as it approaches the setpoint.Unchecked momentum causes overshoot at the end of a dive or turn.Amplifies high-frequency sensor noise, causing rapid thruster oscillation/chatter.6. Autonomous Mission ManagementA. Closed-Loop Mission Execution FlowThe autonomous mission executive coordinates perception, estimation, planning, and control across an 8-stage lifecycle while respecting the safety gates of mission_state_machine.py:Plaintext[1. MISSION START] 
   │  Arms vehicle (/auv/arm) and transitions FSM to MISSION_ACTIVE (/auv/mission_command: START)
   ▼
[2. TARGET SEARCH] 
   │  Commands PID dive to operating depth (2.0m) and executes forward search sweep
   ▼
[3. TARGET DETECTION] 
   │  Triggered when vision_node publishes TargetDetection.detected == True on /auv/vision/detection
   ▼
[4. TARGET POSITION ESTIMATION] 
   │  Projects camera bearing/elevation and range from filtered AUV pose into 3D world coordinates
   ▼
[5. PATH PLANNING] 
   │  Computes collision-free, smoothed waypoints to target via /auv/set_navigation_target
   ▼
[6. NAVIGATION] 
   │  control_node tracks depth, yaw, and surge velocities until planar & depth errors < tolerance
   ▼
[7. TARGET REACHED] 
   │  Maintains station-keeping within target tolerance radius (0.3m)
   ▼
[8. MISSION COMPLETION] 
      Zeroes thruster commands, logs mission summary, and transitions vehicle to standby/return
B. Safety Interlocks & Fault HandlingArming & Active-State Gatekeeping: control_node.py monitors /auv/status (SystemStatus.msg). If the vehicle is DISARMED or not in MISSION_ACTIVE / RETURNING, the PID controllers immediately reset their integrals and publish a zero Twist command so thrusters never spin unexpectedly.Emergency Abort & Return-to-Surface: Triggering ABORT or RETURN on /auv/mission_command overrides active mission goals, commands target depth to 0.0 m, and surfaces the vehicle safely.Ground Station Comms Watchdog (comms_monitor.py): Continuously monitors /auv/heartbeat and /auv/telemetry; if message age exceeds 3.0 s, the operator GUI raises an immediate COMMS LOST alert.7. Build, Verification & Execution CommandsA. Build & Run Automated Unit Test SuiteAll core modules (mission_state_machine, vehicle_sim_model, comms_monitor, navigation_math, state_estimator, color_detector, image_conversion, and pid_controller) are covered by pytest unit tests:Bashcd ~/AUV_int_task-2026
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run unit tests across all packages
pytest src/auv_vehicle/test \
       src/auv_ground_station/test \
       src/auv_navigation/test \
       src/auv_perception/test \
       src/auv_control/test
B. One-Command Autonomy BringupBash# Full stack with Gazebo Harmonic + Vision + Navigation + PID Control + Ground Station GUI
ros2 launch auv_bringup autonomy.launch.py initial_armed:=true

# Headless Gazebo mode (for CI or systems without OpenGL/EGL display)
ros2 launch auv_bringup autonomy.launch.py initial_armed:=true headless:=true

# Lightweight Plant-Model mode (no Gazebo required)
ros2 launch auv_bringup autonomy.launch.py use_gazebo:=false initial_armed:=true
C. Runtime Inspection & Target CommandingBash# Monitor filtered navigation state & errors
ros2 topic echo /auv/navigation/state --once

# Monitor live computer-vision detections
ros2 topic echo /auv/vision/detection --once

# Dynamically command a new 3D target (x, y, depth)
ros2 service call /auv/set_navigation_target auv_interfaces/srv/SetNavigationTarget "{x: 8.0, y: 2.0, depth: 2.5}"

