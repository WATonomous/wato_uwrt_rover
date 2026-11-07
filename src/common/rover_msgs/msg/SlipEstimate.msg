# Wheel slip over a sliding window: slip = 1 - vo_distance / wheel_distance.
std_msgs/Header header
bool valid  # false until enough wheel distance has accumulated with valid VO
float32 slip
float32 wheel_distance
float32 vo_distance
