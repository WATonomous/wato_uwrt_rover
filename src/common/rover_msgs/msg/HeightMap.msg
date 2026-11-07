# Rover-centric 2.5D height map built from navcam depth (ENav's terrain model).
# Cell (i, j) covers [origin_x + i * resolution, origin_x + (i + 1) * resolution) in x, same in y.
std_msgs/Header header  # frame_id is the odom frame
float32 resolution
uint32 width
uint32 height
float64 origin_x
float64 origin_y
float32[] elevation  # row-major (index = j * width + i), top surface height, NaN = never observed
