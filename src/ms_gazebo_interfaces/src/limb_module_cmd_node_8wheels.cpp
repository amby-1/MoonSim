// This program is made based on the example code of Jijin san 
//  THis program is to control the limb module using the joint_cmd_list message.
//
// 2025.1.10 This is example program for 8 wheels robot 
//  This program sense the position of the robot and calculate the joint angles for the wheels

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
#include <nav_msgs/msg/odometry.hpp>  // Include the header for Odometry messages


// for unified ros msg
#include "utility/ms_module_msgs_util.h"
#include "utility/ros2_service_util.h"
#include "sensor_msgs/msg/joint_state.hpp"
#define PI 3.14159265358979323846

// ** ros define
std::shared_ptr<rclcpp::Node> node_;
// ** pub, msg define & func
std::array<rclcpp::Publisher<ms_module_msgs::msg::JointCmdList>::SharedPtr, 8> limb_x_joint_cmd_list_pubs;
std::array<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr, 8> limb_x_joint_state_list_subs;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;  // Define the subscriber for Odometry

// define some global variables
//  these variables are used for the robot position and velocity
//  Variables are updated automatically by the odometory callback function which subscrives Odometry message
std::array<double, 2> robot_position = {0.0, 0.0};  // x, y
std::array<double, 2> robot_velocity = {0.0, 0.0};  // vx, vy

// calc joint angle from position
// This calculate the joint angles from the tip position of the limb module
void calc_joint_angle_from_position(double* position, double* ans_joint_angle){
    double l2 = 0.28;
    double l3 = 0.28;
    double psi = atan2(position[1], -position[2]);
    double b = position[0];
    double a = - position[2] * cos(psi);
    double k = sqrt( pow( (a*a + b*b +l2*l2 + l3*l3), 2) - 2*(  pow(a*a + b*b, 2) + pow(l2,4) + pow(l3, 4)  ) ) ;
    ans_joint_angle[0] = - psi;
    ans_joint_angle[1] = atan2(a,b) -atan2(k, (a*a+b*b+l2*l2-l3*l3) );
    ans_joint_angle[2] = atan2(k, (a*a+b*b - l2*l2-l3*l3) );
    return;
}

void change_range_of_angle(double& angle){
    while(angle > 2*PI){
        angle -= 2*PI;
    }
    while(angle < 0.){
        angle += 2*PI;
    }
    return;
}


// *** func of auto spin
void auto_spin(const std::shared_ptr<rclcpp::Node>& node){
    rclcpp::spin(node);
}


// Callback function for JointState messages
//  If you want to check the joint state of the limb module, you can use this function
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    //RCLCPP_INFO(node_->get_logger(), "Received JointState message");
    for (size_t i = 0; i < msg->name.size(); ++i) {
        //RCLCPP_INFO(node_->get_logger(), "Joint: %s, Position: %f, Velocity: %f, Effort: %f",
        //            msg->name[i].c_str(), msg->position[i], msg->velocity[i], msg->effort[i]);
    }
}

// Callback function for Odometry messages
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_position[0] = msg->pose.pose.position.x;
    robot_position[1] = msg->pose.pose.position.y;
    robot_velocity[0] = msg->twist.twist.linear.x;
    robot_velocity[1] = msg->twist.twist.linear.y;
    RCLCPP_INFO(node_->get_logger(), "Received Odometry message");
    //RCLCPP_INFO(node_->get_logger(), "Position: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    //RCLCPP_INFO(node_->get_logger(), "Orientation: [%f, %f, %f, %f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    RCLCPP_INFO(node_->get_logger(), "Linear Velocity: [%f, %f, %f]", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    //RCLCPP_INFO(node_->get_logger(), "Angular Velocity: [%f, %f, %f]", msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
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
    // Publishers for limbs_1-8 
    limb_x_joint_cmd_list_pubs.at(0) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_1/joint/in/joint_cmd_list", 5); 
    limb_x_joint_cmd_list_pubs.at(1) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_2/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(2) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_3/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(3) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_4/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(4) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_5/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(5) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_6/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(6) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_7/joint/in/joint_cmd_list", 5);
    limb_x_joint_cmd_list_pubs.at(7) = node_->create_publisher<ms_module_msgs::msg::JointCmdList>("/limb_n_8/joint/in/joint_cmd_list", 5);

   // Subscribers for limbs 1-8 
    limb_x_joint_state_list_subs.at(0) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_1/joint/out/joint_state", 10, joint_state_callback);
    limb_x_joint_state_list_subs.at(1) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_2/joint/out/joint_state", 10, joint_state_callback);
    limb_x_joint_state_list_subs.at(2) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_3/joint/out/joint_state", 10, joint_state_callback);
    limb_x_joint_state_list_subs.at(3) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_4/joint/out/joint_state", 10, joint_state_callback);
    limb_x_joint_state_list_subs.at(4) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_5/joint/out/joint_state", 10, joint_state_callback);
    limb_x_joint_state_list_subs.at(5) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_6/joint/out/joint_state", 10, joint_state_callback);
    limb_x_joint_state_list_subs.at(6) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_7/joint/out/joint_state", 10, joint_state_callback);
    limb_x_joint_state_list_subs.at(7) = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb_n_8/joint/out/joint_state", 10, joint_state_callback);

    // Subscriber for /odom
    odom_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odom_callback);

    // *** auto spin in a separate thread. remember= to join()
    std::thread auto_spin_thread(auto_spin, node_);
    // *** ros msg
    // *** define publisher msgs
    SModuleID publisher_id_ = create_module_id("pnp_server", "n", 1);
    std::array<std::array<CModule_Joint_CMD, 4>, 8> limb_joint_cmd;
    for(int i=0; i<8; i++){
        for(int j=0; j<4; j++){
            limb_joint_cmd.at(i).at(j) = CModule_Joint_CMD(publisher_id_, "limb", "n", i+1, "", "", j+1);
        }
    }
    // limb_joint_cmd.at(i).at(j) : Limb i+1, Joint j+1 
        
    // *** define publisher msgs 
    std::array<ms_module_msgs::msg::JointCmdList, 8> cmds;
    for (int i=0; i<8; i++){
        // For the joint 1, 2, 3, 4 of each limb (Position control)
        for (int j=0; j<4; j++){
            double limb_joint_target = 0.0; //deg
            auto limb_joint_cmd_msg = limb_joint_cmd.at(i).at(j).create_position_cmd_msg(node_, limb_joint_target);
            cmds.at(i).joint_cmd_list.push_back(limb_joint_cmd_msg);
        }
    }
    for(int i=0; i<8; i++){
        limb_x_joint_cmd_list_pubs.at(i)->publish(cmds.at(i)); // First, just publish to set zero position. 
    }

    // Stand up the robot
    // 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stand up the robot");
    double standup_time = 15.0; // s
    double targ_CT_angle = 90.0; //deg
    double targ_FT_angle = -90.0; //deg
    for (int i=0; i<standup_time/0.05; i++){
        // *** the main loop
        //  Input joint angles and send them out to
        double CT_angle = targ_CT_angle * i/(standup_time/0.05); // degree
        double FT_angle = targ_FT_angle * i/(standup_time/0.05); // degree
        // limb 1 - 8
        // we set the joint angles of each limb module
        for (int j=0; j<8; j++){
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, 0); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, CT_angle); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, FT_angle); // Limb j+1 Joint3 
        }
        // send 
        for(int j=0; j<8; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
        }
        // wait for 50ms 
        // This imitate the actual robot control frequency 20 Hz
        rclcpp::WallRate rate(50ms);
        rate.sleep();
    }

    // MOVE untilt the robot collides with the step
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move the robot");
    double speed = 60; // deg/s
    double time = 90;
    for (int i=0; i<time/0.05; i++){
        // detect whether the robot is moving forward or stop here 
        if ( i > 20 && fabs(robot_velocity[0]) < 0.01 && fabs(robot_velocity[1]) < 0.01){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DETECT Stop of the robot");
            break;
        }

        // we set the wheel command 
        // cmd for limb 1 and 2 (right side)
        for (int j=0; j<2; j++){ 
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, -speed); // Limb j+1 Joint4 
        }
        // cmd for limb 3 and 4 (left side)
        for (int j=2; j<4; j++){
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, speed); // Limb j+1 Joint4 
        }
        // cmd for limb 5 and 6 (right side)
        for (int j=4; j<6; j++){
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, -speed); // Limb j+1 Joint4 
        }
        // cmd for limb 7 and 8 (left side)
        for (int j=6; j<8; j++){
            cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, speed); // Limb j+1 Joint4 
        }
    
        // send 
        for(int j=0; j<8; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); //  
        }
        rclcpp::WallRate rate(50ms);
        rate.sleep();
    }

    // STOP
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop the robot");
    for (int j=0; j<8; j++){
        cmds.at(j).joint_cmd_list.at(3) = limb_joint_cmd.at(j).at(3).create_velocity_cmd_msg(node_, 0); // Limb j+1 Joint4 
    }
    for(int j=0; j<8; j++){
        limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); 
    }
    // *** ros shutdown
    rclcpp::shutdown();
    return 0;
}
