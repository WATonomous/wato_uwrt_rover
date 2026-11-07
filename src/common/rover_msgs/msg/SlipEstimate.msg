# Wheel slip over a sliding window: slip = 1 - vo_distance / wheel_distance, using signed
# forward distances. 0 = rolling cleanly, 1 = wheels spinning in place, > 1 = sliding back while
# the wheels drive forward, < 0 = sliding forward faster than the wheels turn.
std_msgs/Header header
bool valid  # false until enough wheel distance has accumulated with valid VO
float32 slip
float32 wheel_distance  # m, signed, over the window
float32 vo_distance  # m, signed forward VO distance over the window
