std_msgs/Header header
ms_module_msgs/ModuleID publisher_id
ms_module_msgs/JointID joint_id
int8 control_type
# which following cmd acts
# 0 position control only
# 1 velocity control only
# 2 effort control only
# 3 position with max velocity of inputed velocity
# 4 position with max effort of inputed effort
# 5 position with max velocity and max effort of input
# 6-9 vacancy
# 10 reset_motor
# 11 clear_error
# 12 torque_enable
# 13 control_enable
# 14 lock
float64 position
float64 velocity
float64 effort
bool reset_motor
bool clear_error
bool torque_enable
bool control_enable
bool lock