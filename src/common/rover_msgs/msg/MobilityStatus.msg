uint8 MODE_IDLE = 0
uint8 MODE_PANORAMA = 1  # rotating in place to image the surroundings
uint8 MODE_DRIVING = 2
uint8 MODE_FAULT = 3  # stopped, waiting for /mobility/clear_fault (or auto-recovery)
uint8 MODE_RECOVERING = 4  # backing out after a fault

uint8 FAULT_NONE = 0
uint8 FAULT_TILT = 1
uint8 FAULT_SUSPENSION = 2
uint8 FAULT_SLIP = 3
uint8 FAULT_STALL = 4
uint8 FAULT_NO_SAFE_PATH = 5

std_msgs/Header header
uint8 mode
uint8 fault
string detail
