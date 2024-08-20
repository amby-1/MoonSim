#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <array>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */


std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

void common_goal_response(
 std::shared_ptr<rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>> goal_handle){
  RCLCPP_DEBUG(
    node->get_logger(), "common_goal_response time: %f",
    rclcpp::Clock().now().seconds());
  //auto goal_handle = future.get();
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("SUCCEEDED result code\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Goal was aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Goal was canceled\n");
      return;
    default:
      printf("Unknown result code\n");
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  std::cout << "feedback->desired.positions :";
  for (auto & x : feedback->desired.positions) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
  std::cout << "feedback->desired.velocities :";
  for (auto & x : feedback->desired.velocities) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("minimal_action");

  std::cout << "node created" << std::endl;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");

  bool response =
    action_client->wait_for_action_server(std::chrono::seconds(1));
  if (!response) {
    throw std::runtime_error("could not get action server");
  }
  std::cout << "Created action server" << std::endl;

  std::vector<std::string> joint_names;
    joint_names.push_back("lr0_tc");
    joint_names.push_back("lr0_ct");
    joint_names.push_back("lr0_ft");
    joint_names.push_back("lr1_tc");
    joint_names.push_back("lr1_ct");
    joint_names.push_back("lr1_ft");
    joint_names.push_back("ll0_tc");
    joint_names.push_back("ll0_ct");
    joint_names.push_back("ll0_ft");
    joint_names.push_back("ll1_tc");
    joint_names.push_back("ll1_ct");
    joint_names.push_back("ll1_ft");
    
    
    // set initial trajectory position 
    auto point0 = trajectory_msgs::msg::JointTrajectoryPoint();
    double angle = 0.;
    point0.positions.resize(12);
    // LR0 
    point0.positions.at(0) = 0.1* angle;//angle;
    point0.positions.at(1) = angle;
    point0.positions.at(2) = angle;
    // LR1 
    point0.positions.at(3) = 0.1* angle;//angle;
    point0.positions.at(4) = angle;
    point0.positions.at(5) = angle;
    // LL0 
    point0.positions.at(6) = 0.1* angle;
    point0.positions.at(7) = -angle;
    point0.positions.at(8) = -angle;
    // LL1 
    point0.positions.at(9) = 0.1* angle; //angle;
    point0.positions.at(10) = -angle;
    point0.positions.at(11) = -angle;
    point0.time_from_start = rclcpp::Duration::from_seconds(0.0);
    
    // set 1st trajectory position 
    auto point1 = trajectory_msgs::msg::JointTrajectoryPoint();
    angle =  1.;
    point1.positions.resize(12);
    // LR0 
    point1.positions.at(0) = -0.1* angle;//angle;
    point1.positions.at(1) = angle;
    point1.positions.at(2) = angle;
    // LR1 
    point1.positions.at(3) = 0.1* angle;//angle;
    point1.positions.at(4) = angle;
    point1.positions.at(5) = angle;
    // LL0 
    point1.positions.at(6) = -0.1* angle;
    point1.positions.at(7) = -angle;
    point1.positions.at(8) = -angle;
    // LL1 
    point1.positions.at(9) = 0.1* angle; //angle;
    point1.positions.at(10) = -angle;
    point1.positions.at(11) = -angle;
    point1.time_from_start = rclcpp::Duration::from_seconds(10.0);
    
    // set 2nd trajectory position 
    auto point2 = trajectory_msgs::msg::JointTrajectoryPoint();
    angle =  1.;
    point2.positions.resize(12);
    // LR0 
    point2.positions.at(0) = -0.2* angle;//angle;
    point2.positions.at(1) = angle;
    point2.positions.at(2) = angle;
    // LR1 
    point2.positions.at(3) = 0.2* angle;//angle;
    point2.positions.at(4) = angle;
    point2.positions.at(5) = angle;
    // LL0 
    point2.positions.at(6) = -0.2* angle;
    point2.positions.at(7) = -angle;
    point2.positions.at(8) = -angle;
    // LL1 
    point2.positions.at(9) = 0.2* angle; //angle;
    point2.positions.at(10) = -angle;
    point2.positions.at(11) = -angle;
    point2.time_from_start = rclcpp::Duration::from_seconds(15.0);

    // set 3rd trajectory position 
    auto point3 = trajectory_msgs::msg::JointTrajectoryPoint();
    angle = 0.;
    point3.positions.resize(12);
    // LR0 
    point3.positions.at(0) = 0.1* angle;//angle;
    point3.positions.at(1) = angle;
    point3.positions.at(2) = angle;
    // LR1 
    point3.positions.at(3) = 0.1* angle;//angle;
    point3.positions.at(4) = angle;
    point3.positions.at(5) = angle;
    // LL0 
    point3.positions.at(6) = 0.1* angle;
    point3.positions.at(7) = -angle;
    point3.positions.at(8) = -angle;
    // LL1 
    point3.positions.at(9) = 0.1* angle; //angle;
    point3.positions.at(10) = -angle;
    point3.positions.at(11) = -angle;
    point3.time_from_start = rclcpp::Duration::from_seconds(25.0);

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    points.push_back(point0);
    points.push_back(point1);
    points.push_back(point2);
    points.push_back(point3);

 
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;

  auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);


  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    return 1;
  }
  RCLCPP_ERROR(node->get_logger(), "send goal call ok :)");


  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }
  RCLCPP_ERROR(node->get_logger(), "Goal was accepted by server");


  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  } 

  std::cout << "async_send_goal" << std::endl;
  rclcpp::shutdown();

  return 0;
}


