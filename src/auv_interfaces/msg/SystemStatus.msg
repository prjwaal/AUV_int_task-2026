# Vehicle/System Status
#
# Low-rate, high-importance status channel published on /auv/status.
# Published with RELIABLE + TRANSIENT_LOCAL (durability) QoS so that a
# Ground Station connecting or reconnecting mid-mission immediately receives
# the current status rather than waiting for the next change event, and so
# that no status transition (e.g. entering STATUS_FAULT) is silently lost
# to a momentary subscriber gap.

uint8 STATUS_DISARMED = 0
uint8 STATUS_ARMED = 1
uint8 STATUS_MISSION_ACTIVE = 2
uint8 STATUS_MISSION_PAUSED = 3
uint8 STATUS_RETURNING = 4
uint8 STATUS_FAULT = 5

std_msgs/Header header
uint8 status
float32 battery_percentage

# Human-readable detail, e.g. "Nominal", "Battery critical (<15%)",
# "Lost heading reference". Intended for direct display on the Ground
# Station status panel.
string status_message
