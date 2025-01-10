cmake_minimum_required(VERSION 3.8)
project(ms_gazebo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ms_module_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

# Add the executable
add_executable(limb_cmd_node src/limb_module_cmd_node.cpp)
add_executable(interface_8leg_node src/interface_8leg.cpp)
add_executable(interface_6wheels_node src/interface_6wheels.cpp)
add_executable(interface_5wheels_node src/interface_5wheels.cpp)
add_executable(limb_cmd_6wheels src/limb_module_cmd_node_6wheels.cpp)
add_executable(limb_cmd_5wheels src/limb_module_cmd_node_5wheels.cpp)
add_executable(limb_cmd_test src/limb_module_cmd_node_1limb_for_test.cpp)
add_executable(limb_cmd_6wheels_posver src/limb_module_cmd_node_6wheels_posver.cpp)
add_executable(limb_cmd_8leg src/limb_module_cmd_node_8leg.cpp)
add_executable(interface_8wheels_node src/interface_8wheels.cpp)
add_executable(limb_cmd_8wheels src/limb_module_cmd_node_8wheels.cpp)

# Link against dependencies
ament_target_dependencies(limb_cmd_node rclcpp std_msgs ms_module_msgs)
ament_target_dependencies(limb_cmd_6wheels rclcpp std_msgs ms_module_msgs)
ament_target_dependencies(limb_cmd_5wheels rclcpp std_msgs ms_module_msgs)
ament_target_dependencies(interface_8leg_node rclcpp std_msgs ms_module_msgs trajectory_msgs)
ament_target_dependencies(interface_6wheels_node rclcpp std_msgs ms_module_msgs trajectory_msgs)
ament_target_dependencies(interface_5wheels_node rclcpp std_msgs ms_module_msgs trajectory_msgs)
ament_target_dependencies(interface_8wheels_node rclcpp std_msgs ms_module_msgs trajectory_msgs)
ament_target_dependencies(limb_cmd_test rclcpp std_msgs ms_module_msgs)
ament_target_dependencies(limb_cmd_6wheels_posver rclcpp std_msgs ms_module_msgs)
ament_target_dependencies(limb_cmd_8leg rclcpp std_msgs ms_module_msgs)
ament_target_dependencies(limb_cmd_8wheels rclcpp std_msgs ms_module_msgs nav_msgs)

install(TARGETS
  limb_cmd_node
  interface_8leg_node
  interface_6wheels_node
  interface_5wheels_node
  interface_8wheels_node
  limb_cmd_6wheels
  limb_cmd_5wheels
  limb_cmd_test
  limb_cmd_6wheels_posver
  limb_cmd_8leg
  limb_cmd_8wheels
  DESTINATION lib/${PROJECT_NAME})

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
