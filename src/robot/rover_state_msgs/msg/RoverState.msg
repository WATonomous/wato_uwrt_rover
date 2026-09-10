# Coarse operating mode of the rover.
#
# Owned and published by state_manager_node on /rover_state with
# transient-local QoS, so late-joining nodes immediately receive the
# current state instead of waiting for the next publish.

# --- state constants ---
uint8 WAIT=0        # autonomy halted; no node may command motion
uint8 CONTROL=1     # autonomy permitted

# Stamped so consumers can detect a stale state publisher.
std_msgs/Header header

# One of the constants above.
uint8 state

# Free-form origin of the most recent transition, e.g. "user", "startup",
# "watchdog". Intended for logs and operator UIs, not for control flow.
string reason
