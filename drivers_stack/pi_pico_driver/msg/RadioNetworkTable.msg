# Full CP/NP/WT table from Pico uplink (POS line). Index i = robot_id (base 0).
std_msgs/Header header
uint8 num_robots
uint8[] cp
uint8[] np
bool[] waiting
