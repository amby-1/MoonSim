cmake_minimum_required(VERSION 3.8)
project(ms_module_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


#declare castom msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbsorberCmd.msg"
  "msg/AbsorberErrorState.msg"
  "msg/AbsorberID.msg"
  "msg/AbsorberState.msg"
  "msg/AllJointState.msg"
  "msg/CameraID.msg"
  "msg/ConnectorCmd.msg"
  "msg/ConnectCmd.msg"
  "msg/ConnectionCmd.msg"
  "msg/ConnectionState.msg"
  "msg/ConnectionOppositeInfo.msg"
  "msg/ConnectorErrorState.msg"
  "msg/ConnectorID.msg"
  "msg/ConnectorState.msg"
  "msg/JointCmd.msg"
  "msg/JointCmdList.msg"
  "msg/JointErrorState.msg"
  "msg/JointErrorStateList.msg"
  "msg/JointID.msg"
  "msg/JointState.msg"
  "msg/LeaderName.msg"
  "msg/LedID.msg"
  "msg/LinkingConnectorsState.msg"
  "msg/LinkingConnectorsCmd.msg"
  "msg/ModuleID.msg"
  "msg/ModuleState.msg"
  "msg/MultiFuncEndEffectorID.msg"
  "msg/MultiFuncEndEffectorCmd.msg"
  "msg/MultiFuncEndEffectorState.msg"
  "msg/MultiFuncEndEffectorErrorState.msg"
  "msg/OppositeConnectorInfo.msg"
  "msg/ReversibleGripperID.msg"
  "msg/ReversibleGripperCmd.msg"
  "msg/ReversibleGripperState.msg"
  "msg/ReversibleGripperErrorState.msg"
  "srv/GetAliveness.srv"
  "srv/GetAlivenessList.srv"
  "srv/GetAllAbsorberErrorState.srv"
  "srv/GetAllAbsorberState.srv"
  "srv/GetAllConnectorErrorState.srv"
  "srv/GetAllConnectorState.srv"
  "srv/GetAllJointErrorState.srv"
  "srv/GetAllJointState.srv"
  "srv/GetLeaderName.srv"
  "srv/GetAllLinkingConnectorsState.srv"
  "srv/GetAllMultiFuncEndEffectorState.srv"
  "srv/GetAllReversibleGripperState.srv"
  "srv/GetRanks.srv"
  "srv/SetAbsorberCmd.srv"
  "srv/SetAbsorberCmdList.srv"
  "srv/SetCameraOnOff.srv"
 # "srv/SetConnectorCmd.srv"
  "srv/SetConnectCmd.srv"
  "srv/SetConnectionCmd.srv"
 # "srv/SetConnectorCmdList.srv"
  "srv/SetConnectCmdList.srv"
  "srv/SetJointCmdList.srv"
  "srv/SetLed.srv"
  "srv/SetLinkingConnectorsCmd.srv"
  "srv/SetOrientation.srv"
  "srv/SetMultiFuncEndEffectorCmd.srv"
  "srv/SetMultiFuncEndEffectorCmdList.srv"
  "srv/SetReversibleGripperCmd.srv"
  "srv/SetReversibleGripperCmdList.srv"
  "action/MonitorConnectError.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
