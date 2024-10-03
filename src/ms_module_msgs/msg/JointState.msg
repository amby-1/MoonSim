std_msgs/Header header
# publisher id is the same as the joint id
ms_module_msgs/JointID joint_id
float64 position
float64 velocity
float64 effort
bool is_healthy
string error_str
string[] error_indexes
int64[] error_values