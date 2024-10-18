// This program is made based on the example code of Jijin san 
//  THis program is to control a limb module using the joint_cmd_list message.
//   This is just check the basic operation of the limb module. 
// 

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
#include <sensor_msgs/msg/joint_state.hpp>

// for unified ros msg
#include "utility/ms_module_msgs_util.h"
#include "utility/ros2_service_util.h"

// ** ros define
std::shared_ptr<rclcpp::Node> node_;
// ** pub, msg define & func
std::array<rclcpp::Publisher<ms_module_msgs::msg::JointCmdList>::SharedPtr, 1> limb_x_joint_cmd_list_pubs;
std::array<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr, 1> limb_x_joint_state_list_subs;
// *** limb ID 
static int limb_id = 4;


// *** func of auto spin
void auto_spin(const std::shared_ptr<rclcpp::Node>& node){
    rclcpp::spin(node);
}

// *** callback function
//  Joint state callback 
//  Each limb module has own joint state topic.  
void limb_4_joint_state_list_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
    // *** print the joint state
    for (int i=0; i< msg->position.size(); i++){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %d : %f", i+1, msg->position.at(i));
    }
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
    // Publishers for limb 4 
    limb_x_joint_cmd_list_pubs.at(0) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_4/joint/in/joint_cmd_list", 5); 

    // Subscribers for limb 4
    limb_x_joint_state_list_subs.at(0) = node_->create_subscription<sensor_msgs::msg::JointState>("/limb_n_4/joint/out/joint_state", 5, limb_4_joint_state_list_callback);

    // *** auto spin in a separate thread. remember= to join()
    std::thread auto_spin_thread(auto_spin, node_);
    // *** ros msg
    // *** define publisher msgs
    SModuleID publisher_id_ = create_module_id("pnp_server", "n", 1);
    std::array<std::array<CModule_Joint_CMD, 4>, 1> limb_joint_cmd;
    for(int i=0; i<1; i++){
        for(int j=0; j<4; j++){
            limb_joint_cmd.at(i).at(j) = CModule_Joint_CMD(publisher_id_, "limb", "n", limb_id, "", "", j+1);
        }
    }
    // limb_joint_cmd.at(i).at(j) : Limb i+1, Joint j+1 
        
    // *** define publisher msgs
    std::array<ms_module_msgs::msg::JointCmdList, 1> cmds;
    for (int i=0; i<1; i++){
        // For the joint 1, 2, 3 of each limb (Position control)
        for (int j=0; j<3; j++){
            double limb_joint_target = 0.0; //deg
            auto limb_joint_cmd_msg = limb_joint_cmd.at(i).at(j).create_position_cmd_msg(node_, limb_joint_target);
            cmds.at(i).joint_cmd_list.push_back(limb_joint_cmd_msg);
        }
        // For the joint 4 of each limb (Velocity control)
        double limb_joint_vel = 0.0; //deg
        auto limb_joint_cmd_msg = limb_joint_cmd.at(i).at(3).create_position_cmd_msg(node_, limb_joint_vel);
        cmds.at(i).joint_cmd_list.push_back(limb_joint_cmd_msg);
    }
    for(int i=0; i<1; i++){
        limb_x_joint_cmd_list_pubs.at(i)->publish(cmds.at(i)); // First, just publish to set zero position. 
    }

    // Stand up the robot
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Checking motions 0 => 30");
    double standup_time = 9; // s
    double targ_CT_angle = 0.0; //deg
    double targ_FT_angle = 90.0; //deg
    double targ_TC_angle = 0.0; //deg
    double targ_WH_angle = 0.0; //deg
    double targ_WH_vel = 10.0; //deg    
    // Basic control frequency is 10Hz
    static int freq = 15; // Hz
    for (int i=0; i<standup_time*freq+1; i++){
        // *** the main loop
        //  Input joint angles and send them out to
        double CT_angle = targ_CT_angle * i/(standup_time * freq); // degree
        double FT_angle = targ_FT_angle * i/(standup_time * freq); // degree
        double TC_angle = targ_TC_angle * i/(standup_time * freq); // degree
        double WH_angle = targ_WH_angle * i/(standup_time * freq); // degree
        // limb 1, 2, 3, 4
        for (int j=0; j<1; j++){
            //cmds.at(j).joint_cmd_list.clear();
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, TC_angle); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, CT_angle); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, FT_angle); // Limb j+1 Joint3 
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_position_cmd_msg(node_, WH_angle); // Limb j+1 Joint4 
            //cmds.at(j).joint_cmd_list.push_back(limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, targ_WH_vel)); // Limb j+1 Joint4 
        }
        // send 
        for(int j=0; j<1; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
        }
        rclcpp::WallRate rate(66ms);
        rate.sleep();
    }

    rclcpp::WallRate rate(5000ms);
    rate.sleep();

/*
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Checking motions 30 => 0");
    for (int i=0; i<standup_time*freq+1; i++){
        // *** the main loop
        //  Input joint angles and send them out to
        double CT_angle = targ_CT_angle - targ_CT_angle * i/(standup_time * freq); // degree
        double FT_angle = targ_FT_angle - targ_FT_angle * i/(standup_time * freq); // degree
        double TC_angle = targ_TC_angle - targ_TC_angle * i/(standup_time * freq); // degree
        double WH_angle = targ_WH_angle - targ_WH_angle * i/(standup_time * freq); // degree        // limb 1, 2, 3, 4
        for (int j=0; j<1; j++){
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, TC_angle); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, CT_angle); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, FT_angle); // Limb j+1 Joint3 
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_position_cmd_msg(node_, WH_angle); // Limb j+1 Joint4 
        }
        // send 
        for(int j=0; j<1; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
        }
        rclcpp::WallRate rate(66ms);
        rate.sleep();
    }

    for (int j=0; j<1; j++){
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, 0.); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, 0.); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, 0.); // Limb j+1 Joint3 
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_position_cmd_msg(node_, 0.0); // Limb j+1 Joint4 
    }
    // send 
    for(int j=0; j<1; j++){
        limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
    }
*/
    /*


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
    */

    // *** ros shutdown
    rclcpp::shutdown();
    return 0;
}
