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
#include "sensor_msgs/msg/joint_state.hpp"
#define PI 3.14159265358979323846

// ** ros define
std::shared_ptr<rclcpp::Node> node_;
// ** pub, msg define & func
std::array<rclcpp::Publisher<ms_module_msgs::msg::JointCmdList>::SharedPtr, 8> limb_x_joint_cmd_list_pubs;
std::array<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr, 8> limb_x_joint_state_list_subs;


// calc joint angle from position
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
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "Received JointState message");
    for (size_t i = 0; i < msg->name.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "Joint: %s, Position: %f, Velocity: %f, Effort: %f",
                    msg->name[i].c_str(), msg->position[i], msg->velocity[i], msg->effort[i]);
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

    // *** auto spin in a separate thread. remember= to join()
    std::thread auto_spin_thread(auto_spin, node_);
    // *** ros msg
    // *** define publisher msgs
    SModuleID publisher_id_ = create_module_id("pnp_server", "n", 1);
    std::array<std::array<CModule_Joint_CMD, 3>, 8> limb_joint_cmd;
    for(int i=0; i<8; i++){
        for(int j=0; j<3; j++){
            limb_joint_cmd.at(i).at(j) = CModule_Joint_CMD(publisher_id_, "limb", "n", i+1, "", "", j+1);
        }
    }
    // limb_joint_cmd.at(i).at(j) : Limb i+1, Joint j+1 
        
    // *** define publisher msgs
    std::array<ms_module_msgs::msg::JointCmdList, 8> cmds;
    for (int i=0; i<8; i++){
        // For the joint 1, 2, 3 of each limb (Position control)
        for (int j=0; j<3; j++){
            double limb_joint_target = 0.0; //deg
            auto limb_joint_cmd_msg = limb_joint_cmd.at(i).at(j).create_position_cmd_msg(node_, limb_joint_target);
            cmds.at(i).joint_cmd_list.push_back(limb_joint_cmd_msg);
        }
    }
    for(int i=0; i<8; i++){
        limb_x_joint_cmd_list_pubs.at(i)->publish(cmds.at(i)); // First, just publish to set zero position. 
    }

    // Stand up the robot
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stand up the robot");
    double standup_time = 15.0; // s
    double targ_CT_angle = 60.0; //deg
    double targ_FT_angle = 30.0; //deg
    for (int i=0; i<standup_time/0.05; i++){
        // *** the main loop
        //  Input joint angles and send them out to
        double CT_angle = targ_CT_angle * i/(standup_time/0.05); // degree
        double FT_angle = targ_FT_angle * i/(standup_time/0.05); // degree
        // limb 1, 2, 3, 4
        for (int j=0; j<8; j++){
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, 0); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, CT_angle); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, FT_angle); // Limb j+1 Joint3 
        }
        // send 
        for(int j=0; j<8; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
        }
        rclcpp::WallRate rate(50ms);
        rate.sleep();
    }

    // Walk 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Walk the robot");
    double walk_time = 90.0; // s
    double omega = PI/4.0; // rad/s
    double dStep = 0.08; // m
    double dHeight = 0.1;
    double beta = 0.8;
    double centerPos[3] = {0.14, 0.0, -0.5};

    for (int i=0; i<walk_time/0.05; i++){
        // *** the main loop
        //  Input joint angles and send them out to
        double phi = omega * i * 0.05;
        // limb 1, 2, 3, 4
        for (int j=0; j<8; j++){
            double position[3] = {centerPos[0], centerPos[1], centerPos[2]};
            if (j == 0 || j==1 || j==4 || j==5){
                double phi2 = phi;
                if(j == 0){
                    phi2 = phi;
                }else if(j == 1){
                    phi2 = phi - beta * 2* PI;
                }else if(j == 4){
                    phi2 = phi - 2 * beta * 2* PI;
                }else{
                    phi2 = phi - 3 * beta * 2* PI;
                }
                
                change_range_of_angle(phi2);
                if(phi2  <  2*beta*PI){
                    position[0] = centerPos[0] ;
                    position[1] = centerPos[1] - dStep * cos( phi2 / (2.*beta) );
                    position[2] = centerPos[2];
                }else{
                    position[0] = centerPos[0] ;
                    position[1] = centerPos[1] + dStep * cos( (phi2 - 2*beta*PI)/(2. - 2.*beta) );
                    position[2] = centerPos[2] + dHeight * sin( (phi2 - 2*beta*PI)/(2. - 2.*beta) );
                }
            }else{
                double phi2 = phi + PI;
                if(j == 2){
                    phi2 = phi + PI;
                }else if(j == 3){
                    phi2 = phi + PI - beta * 2* PI;
                }else if(j == 6){
                    phi2 = phi + PI - 2 * beta * 2* PI;
                }else{
                    phi2 = phi + PI - 3 * beta * 2* PI;
                }
                change_range_of_angle(phi2);
                if(phi2  <  2*beta*PI){
                    position[0] = centerPos[0] ;
                    position[1] = centerPos[1] + dStep * cos( phi2 / (2.*beta) );
                    position[2] = centerPos[2];
                }else{
                    position[0] = centerPos[0] ;
                    position[1] = centerPos[1] - dStep * cos( (phi2 - 2*beta*PI)/(2. - 2.*beta) );
                    position[2] = centerPos[2] + dHeight * sin( (phi2 - 2*beta*PI)/(2. - 2.*beta) );
                }
            }
            double joint_angle[3];
            calc_joint_angle_from_position(position, joint_angle);
            cmds.at(j).joint_cmd_list.at(0) = limb_joint_cmd.at(j).at(0).create_position_cmd_msg(node_, joint_angle[0] * 180./PI); // Limb j+1 Joint1 
            cmds.at(j).joint_cmd_list.at(1) = limb_joint_cmd.at(j).at(1).create_position_cmd_msg(node_, joint_angle[1] * 180./PI); // Limb j+1 Joint2 
            cmds.at(j).joint_cmd_list.at(2) = limb_joint_cmd.at(j).at(2).create_position_cmd_msg(node_, joint_angle[2] * 180./PI); // Limb j+1 Joint3 
        }
        // send 
        for(int j=0; j<8; j++){
            limb_x_joint_cmd_list_pubs.at(j)->publish(cmds.at(j)); // First, just publish to set zero position. 
        }
        rclcpp::WallRate rate(50ms);
        rate.sleep();
    }
 
    // *** ros shutdown
    rclcpp::shutdown();
    return 0;
}
