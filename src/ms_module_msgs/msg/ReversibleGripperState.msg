std_msgs/Header header
ms_module_msgs/ReversibleGripperID rev_gripper_id
int8 linked_side #0, not linking, 1, A side, -1, B side
int8 finger_direction #0, neutral, 1, A side, 2, B side
float64 gripping_rate
float64 effort
bool is_healthy
string error_str
string[] error_indexes
int64[] error_values