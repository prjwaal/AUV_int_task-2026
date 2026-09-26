"""
Shared QoS profile definitions for the AUV_int_task-2026 stack.

These live in auv_vehicle (rather than being duplicated per-package) because
every node that talks to the vehicle -- telemetry publisher, command
interface, ground station -- must agree on the same profile per topic or
publisher/subscriber matching silently fails on incompatible QoS.

Rationale
---------
TELEMETRY_QOS (topic: /auv/telemetry, ~20 Hz)
    BEST_EFFORT + VOLATILE + KEEP_LAST(5). Losing an occasional sample is
    fine -- a fresher one follows in <=50ms -- and we do not want a slow
    subscriber (e.g. a GUI redraw) to apply backpressure to the publisher,
    nor do we want retransmission overhead on what will eventually be a
    lossy acoustic/serial link to real hardware.

STATUS_QOS (topic: /auv/status, ~2 Hz)
    RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1). Status changes (e.g. entering
    STATUS_FAULT) must never be silently dropped, and a Ground Station that
    connects or reconnects mid-mission must see the *current* status
    immediately rather than waiting for the next change event -- that is
    exactly what TRANSIENT_LOCAL durability gives a late-joining subscriber.

HEARTBEAT_QOS (topic: /auv/heartbeat, ~1 Hz)
    RELIABLE + VOLATILE + KEEP_LAST(1). Only the most recent heartbeat ever
    matters, but heartbeat *loss* is the actual signal the Ground Station is
    watching for, so the transport should not silently drop a heartbeat and
    mask a real comms interruption.

COMMAND_QOS (topic/service base for arm/disarm, set-target-depth, etc.)
    RELIABLE + VOLATILE + KEEP_LAST(10). Commands must never be silently
    dropped, but a late-joining subscriber should not replay a stale
    arm/disarm command issued minutes earlier.
"""

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

TELEMETRY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

STATUS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

HEARTBEAT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
