std_msgs/Header header
# publisher id is the same as the module_id
ms_module_msgs/ModuleID module_id
ms_module_msgs/JointState[] joints_state_list
ms_module_msgs/ConnectorState[] connectors_state_list
ms_module_msgs/AbsorberState[] absorber_state_list
ms_module_msgs/ConnectionState[] connection_state_list
geometry_msgs/Pose robot_pose
sensor_msgs/BatteryState battery_state
bool is_healthy
string error_str
string[] error_indexes
int64[] error_values