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
  : Node("minimal_publisher"), count_(0)
  {
    // define publisher joint trajectroy message
    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

    auto timer_callback =
      [this]() -> void {
        auto message = trajectory_msgs::msg::JointTrajectory(); //  std_msgs::msg::String();
        //message.header.stamp = this->now();
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
        
        count_++;
        double omega = 0.05;
        double amplitude = 1.;
        double angle = amplitude*sin(count_ / 10. * omega);
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions.resize(12);

        // LR0 
        point.positions.at(0) = 0.;//angle;
        point.positions.at(1) = angle;
        point.positions.at(2) = angle;
        // LR1 
        point.positions.at(3) = 0.;//angle;
        point.positions.at(4) = angle;
        point.positions.at(5) = angle;
        // LL0 
        point.positions.at(6) = 0.;
        point.positions.at(7) = -angle;
        point.positions.at(8) = -angle;
        // LL1 
        point.positions.at(9) = 0.; //angle;
        point.positions.at(10) = -angle;
        point.positions.at(11) = -angle;

        point.time_from_start = rclcpp::Duration::from_seconds(0.01);
        message.points.push_back(point);
        
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f' rad", angle);
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(10ms, timer_callback);
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
