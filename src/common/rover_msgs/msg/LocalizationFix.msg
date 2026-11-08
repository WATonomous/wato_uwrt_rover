# Result of one orbital-map matching attempt by global localization.
std_msgs/Header header
bool accepted
float32 dx  # correction applied to map->odom, m (map frame)
float32 dy
float32 dyaw  # rad
float32 score  # normalized cross-correlation of the best match
float32 runner_up  # best score at least peak_separation away from the best match
uint32 points
