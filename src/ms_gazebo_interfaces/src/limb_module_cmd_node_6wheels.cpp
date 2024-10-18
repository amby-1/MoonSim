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
std::array<rclcpp::Publisher<ms_module_msgs::msg::JointCmdList>::SharedPtr, 6> limb_x_joint_cmd_list_pubs;

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
    // Publishers for limbs_1-6 
    limb_x_joint_cmd_list_pubs.at(0) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_1/joint/in/joint_cmd_list", 5); 
    limb_x_joint_cmd_list_pubs.at(1) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_2/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(2) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_3/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(3) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_4/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(4) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_5/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(5) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_6/joint/in/joint_cmd_list", 5);

    // *** auto spin in a separate thread. remember= to join()
    std::thread auto_spin_thread(auto_spin, node_);
    // *** ros msg
    // *** define publisher msgs
    SModuleID publisher_id_ = create_module_id("pnp_server", "n", 1);
    std::array<std::array<CModule_Joint_CMD, 4>, 6> limb_joint_cmd;
    for(int i=0; i<6; i++){
        for(int j=0; j<4; j++){
            limb_joint_cmd.at(i).at(j) = CModule_Joint_CMD(publisher_id_, "limb", "n", i+1, "", "", j+1);
        }
    }
    // limb_joint_cmd.at(i).at(j) : Limb i+1, Joint j+1 
        
    // *** define publisher msgs
    std::array<ms_module_msgs::msg::JointCmdList, 6> cmds;
    for (int i=0; i<6; i++){
        // For the joint 1, 2, 3 of each limb (Position control)
        for (int j=0; j<3; j++){
            double limb_joint_target = 0.0; //deg
            auto limb_joint_cmd_msg = limb_joint_cmd.at(i).at(j).create_position_cmd_msg(node_, limb_joint_target);
            cmds.at(i).joint_cmd_list.push_back(limb_joint_cmd_msg);
        }
        // For the joint 4 of each limb (Velocity control)
        double limb_joint_vel = 0.0; //deg
        auto limb_joint_cmd_msg = limb_joint_cmd.at(i).at(3).create_velocity_cmd_msg(node_, limb_joint_vel);
        cmds.at(i).joint_cmd_list.push_back(limb_joint_cmd_msg);
    }
    for(int i=0; i<6; i++){
        limb_x_joint_cmd_list_pubs.at(i)->publish(cmds.at(i)); // First, just publish to set zero position. 
    }

    // Stand up the robot
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stand up the robot");
    double standup_time = 10.0; // s
    double targ_CT_angle = 60.0; //deg
    double targ_FT_angle = -60.0; //deg
    for (int i=0; i<standup_time/0.05; i++){
        // *** the main loop
        //  Input joint angles and send them out to
        double CT_angle = targ_CT_angle * i/(standup_time/0.05); // degree
        double FT_angle = targ_FT_angle * i/(standup_time/0.05); // degree
        // limb 1, 2, 3, 4
        for (int j=0; j<4; j++){
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, 0); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, CT_angle); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, FT_angle); // Limb j+1 Joint3 
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, 0.0); // Limb j+1 Joint4 
        }
        // limb 5, 6 Nothing to do
        for (int j=4; j<6; j++){
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, 0); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, 0.); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, 0.); // Limb j+1 Joint3 
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, 0.0); // Limb j+1 Joint4 
        }
        // send 
        for(int j=0; j<6; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
        }
        rclcpp::WallRate rate(50ms);
        rate.sleep();
    }

    // Velocity Control (for the joint 4 of each limb)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Velocity Control");
    double speed = 60; // deg/s
    double time = 10;
    for (int i=0; i<time/0.05; i++){
        // *** the main loop
        //  Input joint angles and send them out to
        // limb 1, 2,
        for (int j=0; j<2; j++){
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, speed); // Limb j+1 Joint4 
        }
        // limb 3, 4
        for (int j=2; j<4; j++){
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, -speed); // Limb j+1 Joint4 
        }
        // limb 5, 6 Nothing to do
        for (int j=4; j<6; j++){
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, 10); // Limb j+1 Joint4 
        }
        for(int j=0; j<6; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
        }
        rclcpp::WallRate rate(50ms);
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop the robot");

    // Stop the robot
    //  Input joint angles and send them out to
    for (int j=0; j<6; j++){
        cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, 0); // Limb j+1 Joint4 
    }

    for(int j=0; j<6; j++){
        limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
    }

    // *** ros shutdown
    rclcpp::shutdown();
    return 0;
}
