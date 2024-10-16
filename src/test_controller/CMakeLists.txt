cmake_minimum_required(VERSION 3.8)
project(test_controller)


#Default to C++14
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
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(talker src/test_command_sender.cpp)
ament_target_dependencies(talker rclcpp std_msgs trajectory_msgs)

add_executable(talker_8leg src/test_command_sender_8leg.cpp)
ament_target_dependencies(talker_8leg rclcpp std_msgs trajectory_msgs)

add_executable(talker_4wheel src/test_command_sender_4wheel.cpp)
ament_target_dependencies(talker_4wheel rclcpp std_msgs trajectory_msgs)

add_executable(talker_6wheel src/test_command_sender_6wheel.cpp)
ament_target_dependencies(talker_6wheel rclcpp std_msgs trajectory_msgs)

add_executable(talker_once src/test_command_sender_once.cpp)
ament_target_dependencies(talker_once rclcpp std_msgs trajectory_msgs)

add_executable(action_once src/test_command_action_once.cpp)
ament_target_dependencies(action_once rclcpp rclcpp_action std_msgs trajectory_msgs control_msgs)


install(TARGETS
  talker
  talker_once
  talker_8leg
  talker_4wheel
  talker_6wheel
  action_once
  DESTINATION lib/${PROJECT_NAME})

ament_package()
