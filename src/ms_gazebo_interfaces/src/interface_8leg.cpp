#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <unordered_map>       // ヘッダファイルインクルード

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "ms_module_msgs/msg/joint_cmd_list.hpp"

using namespace std::chrono_literals;
#define PI 3.14159265358979323846
#define CONTROL_ms 20ms // THis specify the control frequency of gazebo. THe joint commmands are sent with this frequency  

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

// 2024.09.25 Designed by Amby
// THis program is to publish joint trajectory message to gazebo at 100Hz for 8 legged robot while receiving joint command from limb_module_cmd message type
//  THis only check the frame_id of the joint_cmd_list message and update the position of the joint in the joint trajectory message
//  Be notice that this program does not check the correctness of the joint_cmd_list message, only position control is implemented
///

// Memo 
//  TODO :: Add joint state publisher to check the joint situations 
//  TODO :: Calculate the efforts based on the PD gain and joint position error  
//

class ms_gz_interface_8leg : public rclcpp::Node
{
public:
  ms_gz_interface_8leg()
  : Node("ms_gz_interface_8leg"), count_(0), map_gzJoint_msJoint(), map_gzJoint_msJoint_direction(), map_msJoint_gzJoint(), map_gzJoint_ID() 
  {
    // gazebo joint vs moon joint name mapping (Please edit depending on the definition)
    map_gzJoint_msJoint["lr0_tc"] = "limb_n_1_J_1";
    map_gzJoint_msJoint_direction["lr0_tc"] = -1;
    map_gzJoint_msJoint["lr0_ct"] = "limb_n_1_J_2";
    map_gzJoint_msJoint_direction["lr0_ct"] = 1;
    map_gzJoint_msJoint["lr0_ft"] = "limb_n_1_J_3";
    map_gzJoint_msJoint_direction["lr0_ft"] = 1;

    map_gzJoint_msJoint["lr1_tc"] = "limb_n_2_J_1";
    map_gzJoint_msJoint_direction["lr1_tc"] = -1;
    map_gzJoint_msJoint["lr1_ct"] = "limb_n_2_J_2";
    map_gzJoint_msJoint_direction["lr1_ct"] = 1;
    map_gzJoint_msJoint["lr1_ft"] = "limb_n_2_J_3";
    map_gzJoint_msJoint_direction["lr1_ft"] = 1;
    
    map_gzJoint_msJoint["lr2_tc"] = "limb_n_5_J_1";
    map_gzJoint_msJoint_direction["lr2_tc"] = -1;
    map_gzJoint_msJoint["lr2_ct"] = "limb_n_5_J_2";
    map_gzJoint_msJoint_direction["lr2_ct"] = 1;
    map_gzJoint_msJoint["lr2_ft"] = "limb_n_5_J_3";
    map_gzJoint_msJoint_direction["lr2_ft"] = 1;
    
    map_gzJoint_msJoint["lr3_tc"] = "limb_n_6_J_1";
    map_gzJoint_msJoint_direction["lr3_tc"] = -1;
    map_gzJoint_msJoint["lr3_ct"] = "limb_n_6_J_2";
    map_gzJoint_msJoint_direction["lr3_ct"] = 1;
    map_gzJoint_msJoint["lr3_ft"] = "limb_n_6_J_3";
    map_gzJoint_msJoint_direction["lr3_ft"] = 1;
    
    map_gzJoint_msJoint["ll0_tc"] = "limb_n_3_J_1";
    map_gzJoint_msJoint_direction["ll0_tc"] = 1;
    map_gzJoint_msJoint["ll0_ct"] = "limb_n_3_J_2";
    map_gzJoint_msJoint_direction["ll0_ct"] = -1;
    map_gzJoint_msJoint["ll0_ft"] = "limb_n_3_J_3";
    map_gzJoint_msJoint_direction["ll0_ft"] = -1;
    
    map_gzJoint_msJoint["ll1_tc"] = "limb_n_4_J_1";
    map_gzJoint_msJoint_direction["ll1_tc"] = 1;
    map_gzJoint_msJoint["ll1_ct"] = "limb_n_4_J_2";
    map_gzJoint_msJoint_direction["ll1_ct"] = -1;
    map_gzJoint_msJoint["ll1_ft"] = "limb_n_4_J_3";
    map_gzJoint_msJoint_direction["ll1_ft"] = -1;
    
    map_gzJoint_msJoint["ll2_tc"] = "limb_n_7_J_1";
    map_gzJoint_msJoint_direction["ll2_tc"] = 1;
    map_gzJoint_msJoint["ll2_ct"] = "limb_n_7_J_2";
    map_gzJoint_msJoint_direction["ll2_ct"] = -1;
    map_gzJoint_msJoint["ll2_ft"] = "limb_n_7_J_3";
    map_gzJoint_msJoint_direction["ll2_ft"] = -1;
    
    map_gzJoint_msJoint["ll3_tc"] = "limb_n_8_J_1";
    map_gzJoint_msJoint_direction["ll3_tc"] = 1;
    map_gzJoint_msJoint["ll3_ct"] = "limb_n_8_J_2";
    map_gzJoint_msJoint_direction["ll3_ct"] = -1;
    map_gzJoint_msJoint["ll3_ft"] = "limb_n_8_J_3"; 
    map_gzJoint_msJoint_direction["ll3_ft"] = -1;
    

    // gazebo joint name vs Joint IDs -- No need to edit this part
    for(auto itr = map_gzJoint_msJoint.begin(); itr != map_gzJoint_msJoint.end(); ++itr) {
        map_msJoint_gzJoint[itr->second] = itr->first;
    }
    map_gzJoint_ID["lr0_tc"] = 1;
    map_gzJoint_ID["lr0_ct"] = 2;
    map_gzJoint_ID["lr0_ft"] = 3;
    map_gzJoint_ID["lr1_tc"] = 4;
    map_gzJoint_ID["lr1_ct"] = 5;
    map_gzJoint_ID["lr1_ft"] = 6;
    map_gzJoint_ID["lr2_tc"] = 7;
    map_gzJoint_ID["lr2_ct"] = 8;
    map_gzJoint_ID["lr2_ft"] = 9;
    map_gzJoint_ID["lr3_tc"] = 10;
    map_gzJoint_ID["lr3_ct"] = 11;
    map_gzJoint_ID["lr3_ft"] = 12;
    map_gzJoint_ID["ll0_tc"] = 13;
    map_gzJoint_ID["ll0_ct"] = 14;
    map_gzJoint_ID["ll0_ft"] = 15;
    map_gzJoint_ID["ll1_tc"] = 16;
    map_gzJoint_ID["ll1_ct"] = 17;
    map_gzJoint_ID["ll1_ft"] = 18;
    map_gzJoint_ID["ll2_tc"] = 19;
    map_gzJoint_ID["ll2_ct"] = 20;
    map_gzJoint_ID["ll2_ft"] = 21;
    map_gzJoint_ID["ll3_tc"] = 22;
    map_gzJoint_ID["ll3_ct"] = 23;
    map_gzJoint_ID["ll3_ft"] = 24;

    // position command message to joint trajectory message
    jPoints.positions.resize(25);
    for (int i = 0; i < 25; i++) {
        jPoints.positions.at(i) = 0.0;
    }
    
    // define publisher joint trajectroy message
    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
    
    subscription_ = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_1/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8leg::topic_callback, this, std::placeholders::_1));

    auto timer_callback =
      [this]() -> void {
        auto message = trajectory_msgs::msg::JointTrajectory(); //  std_msgs::msg::String();
        //message.header.stamp = this->now();
        message.joint_names.push_back("torso"); // We need to command to torso joint as well because this is defined as joint to measure the torque on the joint. 

        message.joint_names.push_back("lr0_tc");
        message.joint_names.push_back("lr0_ct");
        message.joint_names.push_back("lr0_ft");
        message.joint_names.push_back("lr1_tc");
        message.joint_names.push_back("lr1_ct");
        message.joint_names.push_back("lr1_ft");
        message.joint_names.push_back("lr2_tc");
        message.joint_names.push_back("lr2_ct");
        message.joint_names.push_back("lr2_ft");
        message.joint_names.push_back("lr3_tc");
        message.joint_names.push_back("lr3_ct");
        message.joint_names.push_back("lr3_ft");

        message.joint_names.push_back("ll0_tc");
        message.joint_names.push_back("ll0_ct");
        message.joint_names.push_back("ll0_ft");
        message.joint_names.push_back("ll1_tc");
        message.joint_names.push_back("ll1_ct");
        message.joint_names.push_back("ll1_ft");
        message.joint_names.push_back("ll2_tc");
        message.joint_names.push_back("ll2_ct");
        message.joint_names.push_back("ll2_ft");
        message.joint_names.push_back("ll3_tc");
        message.joint_names.push_back("ll3_ct");
        message.joint_names.push_back("ll3_ft");
        
        count_++;

        /*
        double omega = 0.05;
        double amplitude = 1.;
        double angle = amplitude*sin(count_ / 10. * omega);
        double angle2 = amplitude*sin(count_ / 10. * omega + 0.1);
        double angle3 = amplitude*sin(count_ / 10. * omega + 0.2);
        double angle4 = amplitude*sin(count_ / 10. * omega + 0.3);
        
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions.resize(25);

        // LR0 
        point.positions.at(0) = 0.;//angle;
        point.positions.at(1) = angle;
        point.positions.at(2) = angle;
        // LR1 
        point.positions.at(3) = 0.;//angle;
        point.positions.at(4) = angle2;
        point.positions.at(5) = angle2;
        // LR2
        point.positions.at(6) = 0.;//angle;
        point.positions.at(7) = angle3;
        point.positions.at(8) = angle3;
        // LR3 
        point.positions.at(9) = 0.;//angle;
        point.positions.at(10) = angle4;
        point.positions.at(11) = angle4;
        
        // LL0 
        point.positions.at(12) = 0.;
        point.positions.at(13) = -angle;
        point.positions.at(14) = -angle;
        // LL1 
        point.positions.at(15) = 0.; //angle;
        point.positions.at(16) = -angle2;
        point.positions.at(17) = -angle2;
        // LL2 
        point.positions.at(18) = 0.;
        point.positions.at(19) = -angle3;
        point.positions.at(20) = -angle3;
        // LL3 
        point.positions.at(21) = 0.; //angle;
        point.positions.at(22) = -angle4;
        point.positions.at(23) = -angle4;
        // torso 
        point.positions.at(24) = angle * 0.3;
        */

        jPoints.time_from_start = rclcpp::Duration::from_seconds(0.01);
        message.points.push_back(jPoints);
        
        //RCLCPP_INFO(this->get_logger(), "Publishing lr0-tc: '%f' rad", jPoints.positions.at(1));
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(CONTROL_ms, timer_callback);
    RCLCPP_INFO(this->get_logger(), "Interface node to bridge GZ and Moon. CMD is sent to GZ every %d ms ", CONTROL_ms);
    RCLCPP_INFO(this->get_logger(), "This interface only provides bridge between joint_angle_commands. Sensing information has not YET been bridged <TODO>", CONTROL_ms);
  }

private:
  void topic_callback(const ms_module_msgs::msg::JointCmdList::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received JointCmdList message");
        // Extract the data from the message and put it into a position commands
        for(std::vector<ms_module_msgs::msg::JointCmd>::size_type i=0; i< msg->joint_cmd_list.size(); i++){
            auto joint_cmd = msg->joint_cmd_list[i];
            auto joint_name = joint_cmd.header.frame_id;
            auto it = map_msJoint_gzJoint.find(joint_name);
            if (it == map_msJoint_gzJoint.end()) {
                RCLCPP_WARN(this->get_logger(), "WARNING :: MS Joint name not found in the map::  %s", joint_name.c_str());
                continue;
            }
            auto joint_name_gz = map_msJoint_gzJoint[joint_name];
            auto joint_id = map_gzJoint_ID[joint_name_gz];
            auto joint_position = joint_cmd.position * PI / 180.;
            jPoints.positions.at(joint_id) = joint_position * map_gzJoint_msJoint_direction[joint_name_gz];
        }
    }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_;
  int count_;
  std::unordered_map<std::string, std::string> map_gzJoint_msJoint;
  std::unordered_map<std::string, int> map_gzJoint_msJoint_direction; // If gzJoit's direction is equal to that of msJoint, set 1. otherwise, set  -1     
  std::unordered_map<std::string, std::string> map_msJoint_gzJoint;
  std::map<std::string, int> map_gzJoint_ID;
  trajectory_msgs::msg::JointTrajectoryPoint jPoints;    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ms_gz_interface_8leg>());
  rclcpp::shutdown();
  return 0;
}
