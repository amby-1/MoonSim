#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <unordered_map>       // ヘッダファイルインクルード

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "ms_module_msgs/msg/joint_cmd_list.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
#define PI 3.14159265358979323846
#define CONTROL_ms 20ms // THis specify the control frequency of gazebo. THe joint commmands are sent with this frequency  
#define CONTROL_freq 50 // Please specify the control frequency of the joint commands -->  1000/CONTROL_ms

#define P_gain 50 // gain of P gain 
#define D_gain 5 // Gain of D gain 


/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

// 2024.10.16 Designed by Amby
// THis program is to publish joint trajectory message to gazebo at 100Hz for 6 wheel-legged robot while receiving joint command from limb_module_cmd message type
//  THis only check the frame_id of the joint_cmd_list message and update the position of the joint in the joint trajectory message for TC CT FT joints 
//  For wheels joints, the velocity is updated based on joint_cmd_list message
//  Be notice that this program does not check the correctness of the joint_cmd_list message, Position control for TC-CT-FT joints and Velocity control for WH joint are implemented
// 

// Memo 
//  TODO :: Add joint state publisher to check the joint situations 
//  TODO :: Calculate the efforts based on the PD gain and joint position error to sense the joint torque 
//

class ms_gz_interface_8wheels : public rclcpp::Node
{
public:
  ms_gz_interface_8wheels()
  : Node("ms_gz_interface_8wheels"), count_(0), map_gzJoint_msJoint(), map_gzJoint_msJoint_direction(), map_msJoint_gzJoint(), map_gzJoint_ID() 
  {
    // gazebo joint vs moon joint name mapping (Please edit depending on the definition)
    map_gzJoint_msJoint["lr0_tc"] = "limb_n_1_J_1";
    map_gzJoint_msJoint_direction["lr0_tc"] = -1;
    map_gzJoint_msJoint["lr0_ct"] = "limb_n_1_J_2";
    map_gzJoint_msJoint_direction["lr0_ct"] = 1;
    map_gzJoint_msJoint["lr0_ft"] = "limb_n_1_J_3";
    map_gzJoint_msJoint_direction["lr0_ft"] = 1;
    map_gzJoint_msJoint["lr0_wh"] = "limb_n_1_J_4";
    map_gzJoint_msJoint_direction["lr0_wh"] = -1;

    map_gzJoint_msJoint["lr1_tc"] = "limb_n_2_J_1";
    map_gzJoint_msJoint_direction["lr1_tc"] = -1;
    map_gzJoint_msJoint["lr1_ct"] = "limb_n_2_J_2";
    map_gzJoint_msJoint_direction["lr1_ct"] = 1;
    map_gzJoint_msJoint["lr1_ft"] = "limb_n_2_J_3";
    map_gzJoint_msJoint_direction["lr1_ft"] = 1;
    map_gzJoint_msJoint["lr1_wh"] = "limb_n_2_J_4";
    map_gzJoint_msJoint_direction["lr1_wh"] = -1;
    
    map_gzJoint_msJoint["lr2_tc"] = "limb_n_5_J_1";
    map_gzJoint_msJoint_direction["lr2_tc"] = -1;
    map_gzJoint_msJoint["lr2_ct"] = "limb_n_5_J_2";
    map_gzJoint_msJoint_direction["lr2_ct"] = 1;
    map_gzJoint_msJoint["lr2_ft"] = "limb_n_5_J_3";
    map_gzJoint_msJoint_direction["lr2_ft"] = 1;
    map_gzJoint_msJoint["lr2_wh"] = "limb_n_5_J_4";
    map_gzJoint_msJoint_direction["lr2_wh"] = -1;
    

    map_gzJoint_msJoint["lr3_tc"] = "limb_n_6_J_1";
    map_gzJoint_msJoint_direction["lr3_tc"] = -1;
    map_gzJoint_msJoint["lr3_ct"] = "limb_n_6_J_2";
    map_gzJoint_msJoint_direction["lr3_ct"] = 1;
    map_gzJoint_msJoint["lr3_ft"] = "limb_n_6_J_3";
    map_gzJoint_msJoint_direction["lr3_ft"] = 1;
    map_gzJoint_msJoint["lr3_wh"] = "limb_n_6_J_4";
    map_gzJoint_msJoint_direction["lr3_wh"] = -1;

    map_gzJoint_msJoint["ll0_tc"] = "limb_n_3_J_1";
    map_gzJoint_msJoint_direction["ll0_tc"] = 1;
    map_gzJoint_msJoint["ll0_ct"] = "limb_n_3_J_2";
    map_gzJoint_msJoint_direction["ll0_ct"] = -1;
    map_gzJoint_msJoint["ll0_ft"] = "limb_n_3_J_3";
    map_gzJoint_msJoint_direction["ll0_ft"] = -1;
    map_gzJoint_msJoint["ll0_wh"] = "limb_n_3_J_4";
    map_gzJoint_msJoint_direction["ll0_wh"] = 1;

    map_gzJoint_msJoint["ll1_tc"] = "limb_n_4_J_1";
    map_gzJoint_msJoint_direction["ll1_tc"] = 1;
    map_gzJoint_msJoint["ll1_ct"] = "limb_n_4_J_2";
    map_gzJoint_msJoint_direction["ll1_ct"] = -1;
    map_gzJoint_msJoint["ll1_ft"] = "limb_n_4_J_3";
    map_gzJoint_msJoint_direction["ll1_ft"] = -1;
    map_gzJoint_msJoint["ll1_wh"] = "limb_n_4_J_4";
    map_gzJoint_msJoint_direction["ll1_wh"] = 1;
    
    map_gzJoint_msJoint["ll2_tc"] = "limb_n_7_J_1";
    map_gzJoint_msJoint_direction["ll2_tc"] = 1;
    map_gzJoint_msJoint["ll2_ct"] = "limb_n_7_J_2";
    map_gzJoint_msJoint_direction["ll2_ct"] = -1;
    map_gzJoint_msJoint["ll2_ft"] = "limb_n_7_J_3";
    map_gzJoint_msJoint_direction["ll2_ft"] = -1;
    map_gzJoint_msJoint["ll2_wh"] = "limb_n_7_J_4";
    map_gzJoint_msJoint_direction["ll2_wh"] = 1;
    
    map_gzJoint_msJoint["ll3_tc"] = "limb_n_8_J_1";
    map_gzJoint_msJoint_direction["ll3_tc"] = 1;
    map_gzJoint_msJoint["ll3_ct"] = "limb_n_8_J_2";
    map_gzJoint_msJoint_direction["ll3_ct"] = -1;
    map_gzJoint_msJoint["ll3_ft"] = "limb_n_8_J_3"; 
    map_gzJoint_msJoint_direction["ll3_ft"] = -1;
    map_gzJoint_msJoint["ll3_wh"] = "limb_n_8_J_4";
    map_gzJoint_msJoint_direction["ll3_wh"] = 1;
  

    for (int i = 0; i < 8; i++){
      joints_WH_positions[i] = 0.0;
    }

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

    map_gzJoint_ID["lr0_wh"] = 25;
    map_gzJoint_ID["lr1_wh"] = 26;
    map_gzJoint_ID["lr2_wh"] = 27;
    map_gzJoint_ID["lr3_wh"] = 28;
    map_gzJoint_ID["ll0_wh"] = 29;
    map_gzJoint_ID["ll1_wh"] = 30;
    map_gzJoint_ID["ll2_wh"] = 31;
    map_gzJoint_ID["ll3_wh"] = 32;


    // position command message to joint trajectory message
    jPoints.positions.resize(33);
    jPoints.velocities.resize(33);
    for (int i = 0; i < 33; i++) {
        jPoints.positions.at(i) = 0.0;
        jPoints.velocities.at(i) = 0.0;
    }
    
    // define publisher joint trajectroy message
    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
    publisher_joint_states.at(0) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_1/joint/out/joint_state", 10);
    publisher_joint_states.at(1) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_2/joint/out/joint_state", 10);
    publisher_joint_states.at(2) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_3/joint/out/joint_state", 10);
    publisher_joint_states.at(3) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_4/joint/out/joint_state", 10);
    publisher_joint_states.at(4) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_5/joint/out/joint_state", 10);    
    publisher_joint_states.at(5) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_6/joint/out/joint_state", 10);    
    publisher_joint_states.at(6) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_7/joint/out/joint_state", 10);    
    publisher_joint_states.at(7) = this->create_publisher<sensor_msgs::msg::JointState>("/limb_n_8/joint/out/joint_state", 10);    
    
    subscription_1 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_1/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));
    subscription_2 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_2/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));
    subscription_3 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_3/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));
    subscription_4 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_4/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));
    subscription_5 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_5/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));
    subscription_6 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_6/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));
    subscription_7 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_7/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));
    subscription_8 = this->create_subscription<ms_module_msgs::msg::JointCmdList>(
            "/limb_n_8/joint/in/joint_cmd_list", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback, this, std::placeholders::_1));

    subscription_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ms_gz_interface_8wheels::topic_callback_joint_state, this, std::placeholders::_1));

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

        message.joint_names.push_back("lr0_wh");
        message.joint_names.push_back("lr1_wh");
        message.joint_names.push_back("lr2_wh");
        message.joint_names.push_back("lr3_wh");
        message.joint_names.push_back("ll0_wh");
        message.joint_names.push_back("ll1_wh");
        message.joint_names.push_back("ll2_wh");
        message.joint_names.push_back("ll3_wh");
            
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
        point.positions.at(6) = 0.;//angle;f
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

    auto timer_callback_joint_state =
      [this]() -> void {
        //Publish joint state message
        for (int i=0; i<8; i++){
            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->now();
            for (int j=0; j<4; j++){
              auto name = "limb_n_" + std::to_string(i+1) + "_J_" + std::to_string(j+1);// limb_n_2_J_4 
              // Joint 1 TC
              message.name.push_back(name);
              message.position.push_back(sns_positions.at(map_gzJoint_ID[ map_msJoint_gzJoint[name] ]));
              message.velocity.push_back(sns_velocities.at(map_gzJoint_ID[ map_msJoint_gzJoint[name] ]));
              message.effort.push_back(sns_efforts.at(map_gzJoint_ID[ map_msJoint_gzJoint[name] ]));
            }
            this->publisher_joint_states.at(i)->publish(message);
        }
      };

    timer_ = this->create_wall_timer(CONTROL_ms, timer_callback);
    timer2_ = this->create_wall_timer(20ms, timer_callback_joint_state);
    RCLCPP_INFO(this->get_logger(), "Interface node to bridge GZ and Moon. CMD is sent to GZ every %d ms ", CONTROL_ms);
    RCLCPP_INFO(this->get_logger(), "This interface only provides bridge between joint_angle_commands. Sensing information has not YET been bridged <TODO>");
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
            double joint_position = 0.0;
            if (joint_name_gz.find("wh") != std::string::npos) {
                if( joint_cmd.control_type == 1){
                  // velocity control
                  joints_WH_positions[joint_id - 24] += joint_cmd.velocity * PI / 180. * 1./CONTROL_freq;
                  joint_position = joints_WH_positions[joint_id - 24];
                  jPoints.velocities.at(joint_id) = joint_cmd.velocity * PI / 180.;
                }else{
                  // position control
                  joint_position = joint_cmd.position * PI / 180.;
                  joints_WH_positions[joint_id - 24] = joint_position;
                }
            }else{
                joint_position = joint_cmd.position * PI / 180.;
            }
            jPoints.positions.at(joint_id) = joint_position * map_gzJoint_msJoint_direction[joint_name_gz];
        }
    }
   
  // joint state callback
  void topic_callback_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Extract the data from the message and put it into a position commands
        for(std::vector<std::string>::size_type i=0; i< msg->name.size(); i++){
            auto joint_name = msg->name[i];
            auto it = map_gzJoint_msJoint.find(joint_name);
            if (it == map_gzJoint_msJoint.end()) {
              if(joint_name != "torso"){
                RCLCPP_WARN(this->get_logger(), "WARNING :: GZ Joint name not found in the map::  %s", joint_name.c_str());
              }
                continue;
            }
            //auto joint_name_ms = map_gzJoint_msJoint[joint_name];
            auto joint_id = map_gzJoint_ID[joint_name];
            sns_positions.at(joint_id) = msg->position[i] * 180./PI * map_gzJoint_msJoint_direction[joint_name];
            sns_velocities.at(joint_id) = msg->velocity[i] * 180./PI * map_gzJoint_msJoint_direction[joint_name];
            double cmd_position = jPoints.positions.at(joint_id);
            double estimated_torque = P_gain * (cmd_position - msg->position[i]) - D_gain*msg->velocity[i];
            sns_efforts.at(joint_id) = estimated_torque * map_gzJoint_msJoint_direction[joint_name];           
            //sns_efforts.at(joint_id) = msg->effort[i] * map_gzJoint_msJoint_direction[joint_name];
        }
    }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_state;

  std::array<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr, 8> publisher_joint_states;
 
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_1;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_2;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_3;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_4;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_5;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_6;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_7;
  rclcpp::Subscription<ms_module_msgs::msg::JointCmdList>::SharedPtr subscription_8;
  
  
  int count_;
  std::unordered_map<std::string, std::string> map_gzJoint_msJoint;
  std::unordered_map<std::string, int> map_gzJoint_msJoint_direction; // If gzJoit's direction is equal to that of msJoint, set 1. otherwise, set  -1     
  std::unordered_map<std::string, std::string> map_msJoint_gzJoint;
  std::map<std::string, int> map_gzJoint_ID;
  trajectory_msgs::msg::JointTrajectoryPoint jPoints;
  std::array<double, 33> sns_positions;
  std::array<double, 33> sns_velocities;
  std::array<double, 33> sns_efforts;

  double joints_WH_positions[8];

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ms_gz_interface_8wheels>());
  rclcpp::shutdown();
  return 0;
}
