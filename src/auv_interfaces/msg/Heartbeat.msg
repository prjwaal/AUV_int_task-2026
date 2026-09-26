# Fixed-rate liveness signal published on /auv/heartbeat at heartbeat_rate_hz
# (default 1 Hz). The Ground Station uses gaps in `sequence` and/or elapsed
# wall-clock time since the last message to flag a communication failure
# independently of whatever the last-known telemetry/status said -- a stale
# but "nominal-looking" status is itself a failure signature this topic is
# designed to catch.

std_msgs/Header header
uint32 sequence
