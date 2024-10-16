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
        //message.header.stamp = this->now();
        message.joint_names.push_back("LR0_TC");
        message.joint_names.push_back("LR0_CT");
        message.joint_names.push_back("LR0_FT");
        message.joint_names.push_back("LR1_TC");
        message.joint_names.push_back("LR1_CT");
        message.joint_names.push_back("LR1_FT");
        message.joint_names.push_back("LL0_TC");
        message.joint_names.push_back("LL0_CT");
        message.joint_names.push_back("LL0_FT");
        message.joint_names.push_back("LL1_TC");
        message.joint_names.push_back("LL1_CT");
        message.joint_names.push_back("LL1_FT");
        message.joint_names.push_back("LR0_WH");
        message.joint_names.push_back("LR1_WH");
        message.joint_names.push_back("LL0_WH");
        message.joint_names.push_back("LL1_WH");
        
        count_++;
        double omega = 0.05;
        double amplitude = 0.3;
        double angle =  amplitude*sin(count_ / 10. * omega);
        double angle2 =  amplitude*sin(count_ / 10. * omega + 0.);
        double angle3 =  amplitude*sin(count_ / 10. * omega + 0.);
        double angle4 =  amplitude*sin(count_ / 10. * omega + 0.);
        double vel = 0.5;
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions.resize(16);
        point.velocities.resize(16);

        // LR0 
        point.positions.at(0) = 0.;
        point.positions.at(1) = -angle;
        point.positions.at(2) = -angle;
        // LR1 
        point.positions.at(3) = 0.;//angle;
        point.positions.at(4) = -angle2;
        point.positions.at(5) = -angle2;
        // LL1
        point.positions.at(6) = 0.;//angle;
        point.positions.at(7) = angle3;
        point.positions.at(8) = angle3;
        // LL2 
        point.positions.at(9) = 0.;//angle;
        point.positions.at(10) = angle4;
        point.positions.at(11) = angle4;
        
        // LL0 
        point.positions.at(12) = vel * count_/100.;//angle;
        point.positions.at(13) = vel * count_/100.;//angle;
        point.positions.at(14) = vel * count_/100.;//angle;
        point.positions.at(15) = vel * count_/100.;//angle;
        point.velocities.at(12) = vel;//angle;
        point.velocities.at(13) = vel;//angle;
        point.velocities.at(14) = vel;//angle;
        point.velocities.at(15) = vel;//angle;

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
