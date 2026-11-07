# Frame-to-frame visual odometry from the navcam.
std_msgs/Header header  # stamp of the current frame
builtin_interfaces/Time previous_stamp
geometry_msgs/Pose delta  # base_link at header.stamp, expressed in base_link at previous_stamp
bool valid
uint32 matches
uint32 inliers
