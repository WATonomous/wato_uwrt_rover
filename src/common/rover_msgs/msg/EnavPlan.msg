# Output of one ENav planning cycle.
uint8 STATUS_IDLE = 0  # no goal
uint8 STATUS_DRIVING = 1  # path is valid, drive it
uint8 STATUS_GOAL_REACHED = 2
uint8 STATUS_NO_SAFE_PATH = 3

std_msgs/Header header  # odom frame
uint8 status
# Chosen path, starting at the rover position with the heading to turn to in place
# (the first pose's orientation), then the arcs that follow.
nav_msgs/Path path
float32 commit_length  # m along path the rover may drive before it needs a newer plan
uint32 candidates  # paths that survived the cheap ranking filters
uint32 ace_evaluations  # paths ACE checked before one passed
float32 planning_time_ms
