// This program is made based on the example code of Jijin san 
//  THis program is to control the limb module using the joint_cmd_list message.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <list>
using namespace std;

// *** ROS include
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

// for unified ros msg
#include "utility/ms_module_msgs_util.h"
#include "utility/ros2_service_util.h"

// ** ros define
std::shared_ptr<rclcpp::Node> node_;
// ** pub, msg define & func
rclcpp::Publisher<ms_module_msgs::msg::JointCmdList>::SharedPtr limb_x_joint_cmd_list_pub_;

// *** func of auto spin
void auto_spin(const std::shared_ptr<rclcpp::Node>& node){
    rclcpp::spin(node);
}
// #############################################
int main(int argc, char * argv[])
{
    // *** init node
    rclcpp::init(argc, argv);
    node_ = rclcpp::Node::make_shared("limb_module_cmd_node");
    // *** activate printf() using launch
    //setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // *** ros define
    // limb_x
    limb_x_joint_cmd_list_pub_ = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_1/joint/in/joint_cmd_list", 5);
    // *** auto spin in a separate thread. remember= to join()
    std::thread auto_spin_thread(auto_spin, node_);
    // *** ros msg
    // *** define publisher msgs
    SModuleID publisher_id_ = create_module_id("pnp_server", "n", 1);
    auto limb_1_joint_1_cmd_ = CModule_Joint_CMD(publisher_id_, "limb", "n", 1, "", "", 1);
    auto limb_1_joint_2_cmd_ = CModule_Joint_CMD(publisher_id_, "limb", "n", 1, "", "", 2);
    auto limb_1_joint_3_cmd_ = CModule_Joint_CMD(publisher_id_, "limb", "n", 1, "", "", 3);

    auto limb_3_joint_1_cmd_ = CModule_Joint_CMD(publisher_id_, "limb", "n", 3, "", "", 1);
    auto limb_3_joint_2_cmd_ = CModule_Joint_CMD(publisher_id_, "limb", "n", 3, "", "", 2);
    auto limb_3_joint_3_cmd_ = CModule_Joint_CMD(publisher_id_, "limb", "n", 3, "", "", 3);
    
    // *** define publisher msgs
    double limb_1_joint_1_target_ = 0.0; //deg
    double limb_1_joint_2_target_ = 0.0; //deg
    double limb_1_joint_3_target_ = 0.0; //deg
    double limb_3_joint_1_target_ = 0.0; //deg
    double limb_3_joint_2_target_ = 0.0; //deg
    double limb_3_joint_3_target_ = 0.0; //deg
    
    auto limb_1_joint_1_cmd_msg = limb_1_joint_1_cmd_.create_position_cmd_msg(node_, limb_1_joint_1_target_);
    auto limb_1_joint_2_cmd_msg = limb_1_joint_2_cmd_.create_position_cmd_msg(node_, limb_1_joint_2_target_);
    auto limb_1_joint_3_cmd_msg = limb_1_joint_3_cmd_.create_position_cmd_msg(node_, limb_1_joint_3_target_);
    auto limb_3_joint_1_cmd_msg = limb_3_joint_1_cmd_.create_position_cmd_msg(node_, limb_3_joint_1_target_);
    auto limb_3_joint_2_cmd_msg = limb_3_joint_2_cmd_.create_position_cmd_msg(node_, limb_3_joint_2_target_);
    auto limb_3_joint_3_cmd_msg = limb_3_joint_3_cmd_.create_position_cmd_msg(node_, limb_3_joint_3_target_);

    ms_module_msgs::msg::JointCmdList cmd;
    cmd.joint_cmd_list.push_back(limb_1_joint_1_cmd_msg);
    cmd.joint_cmd_list.push_back(limb_1_joint_2_cmd_msg);
    cmd.joint_cmd_list.push_back(limb_1_joint_3_cmd_msg);
    cmd.joint_cmd_list.push_back(limb_3_joint_1_cmd_msg);
    cmd.joint_cmd_list.push_back(limb_3_joint_2_cmd_msg);
    cmd.joint_cmd_list.push_back(limb_3_joint_3_cmd_msg);
    limb_x_joint_cmd_list_pub_->publish(cmd); // First, just publish to set zero position. 

    // *** the main loop
    double time = 0;
    double omega = 2.0;
    try{
        rclcpp::WallRate rate(50ms);
        while(rclcpp::ok()){
            // *** the main loop
            //  Input joint angles and send them out to the Gazebo
            double sholderAngle = 0. * sin(omega * time); // degree
            double jointAngle = 30. * sin(omega * time); // degree
            cmd.joint_cmd_list.at(0) = limb_1_joint_1_cmd_.create_position_cmd_msg(node_, sholderAngle); // Limb1 Joint1 
            cmd.joint_cmd_list.at(1) = limb_1_joint_2_cmd_.create_position_cmd_msg(node_, jointAngle); // Limb1 Joint2 
            cmd.joint_cmd_list.at(2)= limb_1_joint_3_cmd_.create_position_cmd_msg(node_, jointAngle); // Limb1 Joint3 
            
            cmd.joint_cmd_list.at(3)= limb_3_joint_1_cmd_.create_position_cmd_msg(node_, sholderAngle); // Limb3 Joint1 
            cmd.joint_cmd_list.at(4)= limb_3_joint_2_cmd_.create_position_cmd_msg(node_, jointAngle); // Limb3 Joint2 
            cmd.joint_cmd_list.at(5)= limb_3_joint_3_cmd_.create_position_cmd_msg(node_, jointAngle); // Limb3 Joint3 

        	limb_x_joint_cmd_list_pub_->publish(cmd);
            rate.sleep();
            time += 0.05;
        }
    }catch(const std::exception& e){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error: '%s'", e.what());
    }
    rclcpp::shutdown();
    auto_spin_thread.join();
    return 0;
}
