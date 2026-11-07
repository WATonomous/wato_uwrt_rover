# Ground-truth comparison, simulation only.
std_msgs/Header header
float32 distance_traveled  # m, true path length
float32 position_error  # m, map-frame estimate vs truth (with global localization)
float32 heading_error  # rad
float32 odom_position_error  # m, odometry alone (initial map->odom kept fixed)
float32 drift_percent  # odom_position_error / distance_traveled * 100
