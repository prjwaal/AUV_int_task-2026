# Vehicle State & Telemetry
#
# High-rate kinematic + power snapshot of the AUV. This is the message
# published on /auv/telemetry at telemetry_rate_hz (default 20 Hz).
#
# `status` mirrors the authoritative, latched channel published on
# /auv/status (see SystemStatus.msg) so that a consumer only interested in
# telemetry does not also need to subscribe to the status topic. The status
# topic remains the source of truth for status *changes* (it is latched via
# TRANSIENT_LOCAL durability so late subscribers get it immediately); this
# field is a best-effort, high-rate echo of the same value.

uint8 STATUS_DISARMED = 0
uint8 STATUS_ARMED = 1
uint8 STATUS_MISSION_ACTIVE = 2
uint8 STATUS_MISSION_PAUSED = 3
uint8 STATUS_RETURNING = 4
uint8 STATUS_FAULT = 5

std_msgs/Header header

# Position in the local ENU world frame (metres). z is up, so a submerged
# vehicle has z = -depth.
geometry_msgs/Point position

# Orientation in the local ENU world frame.
geometry_msgs/Quaternion orientation

# Depth below the surface (metres, always >= 0). Kept as its own field
# (rather than derived from -position.z) because depth is usually measured
# directly by a pressure sensor and is the primary control/display variable.
float64 depth

# Body-frame linear and angular velocity (m/s, rad/s).
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 angular_velocity

float32 battery_voltage
float32 battery_percentage

uint8 status
