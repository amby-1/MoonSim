#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher2"), count_(0)
  {
    // define publisher joint trajectroy message
    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

    auto timer_callback =
      [this]() -> void {
        auto message = trajectory_msgs::msg::JointTrajectory(); //  std_msgs::msg::String();
        message.header.stamp = this->now();
        message.joint_names.push_back("lr0_tc");
        message.joint_names.push_back("lr0_ct");
        message.joint_names.push_back("lr0_ft");
        message.joint_names.push_back("lr1_tc");
        message.joint_names.push_back("lr1_ct");
        message.joint_names.push_back("lr1_ft");
        message.joint_names.push_back("ll0_tc");
        message.joint_names.push_back("ll0_ct");
        message.joint_names.push_back("ll0_ft");
        message.joint_names.push_back("ll1_tc");
        message.joint_names.push_back("ll1_ct");
        message.joint_names.push_back("ll1_ft");
        
        
        // set initial trajectory position 
        auto point0 = trajectory_msgs::msg::JointTrajectoryPoint();
        double angle = 0.;
        point0.positions.resize(12);
        // LR0 
        point0.positions.at(0) = 0.;//angle;
        point0.positions.at(1) = angle;
        point0.positions.at(2) = angle;
        // LR1 
        point0.positions.at(3) = 0.;//angle;
        point0.positions.at(4) = angle;
        point0.positions.at(5) = angle;
        // LL0 
        point0.positions.at(6) = 0.;
        point0.positions.at(7) = -angle;
        point0.positions.at(8) = -angle;
        // LL1 
        point0.positions.at(9) = 0.; //angle;
        point0.positions.at(10) = -angle;
        point0.positions.at(11) = -angle;
        point0.time_from_start = rclcpp::Duration::from_seconds(0.0);
        
        // set 1st trajectory position 
        auto point1 = trajectory_msgs::msg::JointTrajectoryPoint();
        angle = - 0.5;
        point1.positions.resize(12);
        // LR0 
        point1.positions.at(0) = 0.;//angle;
        point1.positions.at(1) = angle;
        point1.positions.at(2) = angle;
        // LR1 
        point1.positions.at(3) = 0.;//angle;
        point1.positions.at(4) = angle;
        point1.positions.at(5) = angle;
        // LL0 
        point1.positions.at(6) = 0.;
        point1.positions.at(7) = -angle;
        point1.positions.at(8) = -angle;
        // LL1 
        point1.positions.at(9) = 0.; //angle;
        point1.positions.at(10) = -angle;
        point1.positions.at(11) = -angle;
        point1.time_from_start = rclcpp::Duration::from_seconds(1.0);
        
        // set 2nd trajectory position 
        auto point2 = trajectory_msgs::msg::JointTrajectoryPoint();
        angle = - 1.;
        point2.positions.resize(12);
        // LR0 
        point2.positions.at(0) = 0.;//angle;
        point2.positions.at(1) = angle;
        point2.positions.at(2) = angle;
        // LR1 
        point2.positions.at(3) = 0.;//angle;
        point2.positions.at(4) = angle;
        point2.positions.at(5) = angle;
        // LL0 
        point2.positions.at(6) = 0.;
        point2.positions.at(7) = -angle;
        point2.positions.at(8) = -angle;
        // LL1 
        point2.positions.at(9) = 0.; //angle;
        point2.positions.at(10) = -angle;
        point2.positions.at(11) = -angle;
        point2.time_from_start = rclcpp::Duration::from_seconds(2.0);

        // set 3rd trajectory position 
        auto point3 = trajectory_msgs::msg::JointTrajectoryPoint();
        angle = 0.;
        point3.positions.resize(12);
        // LR0 
        point3.positions.at(0) = 0.;//angle;
        point3.positions.at(1) = angle;
        point3.positions.at(2) = angle;
        // LR1 
        point3.positions.at(3) = 0.;//angle;
        point3.positions.at(4) = angle;
        point3.positions.at(5) = angle;
        // LL0 
        point3.positions.at(6) = 0.;
        point3.positions.at(7) = -angle;
        point3.positions.at(8) = -angle;
        // LL1 
        point3.positions.at(9) = 0.; //angle;
        point3.positions.at(10) = -angle;
        point3.positions.at(11) = -angle;
        point3.time_from_start = rclcpp::Duration::from_seconds(4.0);


        message.points.push_back(point0);
        message.points.push_back(point1);
        message.points.push_back(point2);
        message.points.push_back(point3);
        
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f' rad", angle);
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(5000ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
