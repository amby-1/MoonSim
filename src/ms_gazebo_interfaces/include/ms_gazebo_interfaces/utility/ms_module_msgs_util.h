#ifndef MS_MODULE_MSGS_ULIL_H_
#define MS_MODULE_MSGS_ULIL_H_

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
using namespace std;

// *** ROS include
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
// *** custom ROS msgs
#include "ms_module_msgs/msg/absorber_cmd.hpp"
#include "ms_module_msgs/msg/absorber_error_state.hpp"
#include "ms_module_msgs/msg/absorber_id.hpp"
#include "ms_module_msgs/msg/absorber_state.hpp"
#include "ms_module_msgs/msg/all_joint_state.hpp"
#include "ms_module_msgs/msg/camera_id.hpp"
#include "ms_module_msgs/msg/connect_cmd.hpp"
#include "ms_module_msgs/msg/connection_state.hpp"
#include "ms_module_msgs/msg/connection_cmd.hpp"
#include "ms_module_msgs/msg/connection_opposite_info.hpp"
#include "ms_module_msgs/msg/connector_error_state.hpp"
#include "ms_module_msgs/msg/connector_id.hpp"
#include "ms_module_msgs/msg/connector_state.hpp"
#include "ms_module_msgs/msg/joint_cmd.hpp"
#include "ms_module_msgs/msg/joint_cmd_list.hpp"
#include "ms_module_msgs/msg/joint_error_state.hpp"
#include "ms_module_msgs/msg/joint_error_state_list.hpp"
#include "ms_module_msgs/msg/joint_id.hpp"
#include "ms_module_msgs/msg/joint_state.hpp"
#include "ms_module_msgs/msg/leader_name.hpp"
#include "ms_module_msgs/msg/led_id.hpp"
#include "ms_module_msgs/msg/module_id.hpp"
#include "ms_module_msgs/msg/module_state.hpp"
// *** custom ROS srvice
#include "ms_module_msgs/srv/get_aliveness.hpp"
#include "ms_module_msgs/srv/get_aliveness_list.hpp"
#include "ms_module_msgs/srv/get_all_absorber_error_state.hpp"
#include "ms_module_msgs/srv/get_all_absorber_state.hpp"
#include "ms_module_msgs/srv/get_all_connector_error_state.hpp"
#include "ms_module_msgs/srv/get_all_connector_state.hpp"
#include "ms_module_msgs/srv/get_all_joint_error_state.hpp"
#include "ms_module_msgs/srv/get_all_joint_state.hpp"
#include "ms_module_msgs/srv/get_leader_name.hpp"
#include "ms_module_msgs/srv/get_ranks.hpp"
#include "ms_module_msgs/srv/set_absorber_cmd.hpp"
#include "ms_module_msgs/srv/set_absorber_cmd_list.hpp"
#include "ms_module_msgs/srv/set_connect_cmd.hpp"
#include "ms_module_msgs/srv/set_connect_cmd_list.hpp"
#include "ms_module_msgs/srv/set_connection_cmd.hpp"
#include "ms_module_msgs/srv/set_joint_cmd_list.hpp"
#include "ms_module_msgs/srv/set_led.hpp"
#include "ms_module_msgs/srv/set_orientation.hpp"
// *** custom ROS action
#include "ms_module_msgs/action/monitor_connect_error.hpp"


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** utilities
string to_string(double num, int pre){
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(pre) << num;
    return oss.str();
}


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** define module
struct SModuleID{
    string module_type_;
    string module_subtype_;
    unsigned int module_num_;
    string id_str_;
    string id_str_short_;
};
bool is_id_same(const SModuleID& sid1, const SModuleID& sid2){
    return(sid1.module_type_ == sid2.module_type_ 
        && sid1.module_subtype_ == sid2.module_subtype_
        && sid1.module_num_ == sid2.module_num_);    
}
bool is_id_same(const ms_module_msgs::msg::ModuleID& sid_msg1, const ms_module_msgs::msg::ModuleID& sid_msg2){
    return(sid_msg1.type == sid_msg2.type 
        && sid_msg1.subtype == sid_msg2.subtype
        && sid_msg1.num == sid_msg2.num);    
}
SModuleID create_module_id(string module_type, string module_subtype, unsigned int module_num){
    if(module_type == ""){
        printf("varible module_type can not empty when create_module_id");
        module_type = "UNKONWN";
    }
    SModuleID sid;
    sid.module_type_ = module_type;
    sid.module_subtype_ = module_subtype;
    sid.module_num_ = module_num;

    sid.id_str_ = module_type;
    if(sid.module_subtype_ != ""){
        sid.id_str_ = sid.id_str_ + "_" + sid.module_subtype_;
    }
    sid.id_str_ = sid.id_str_ + "_" + to_string(sid.module_num_);

    sid.id_str_short_ = module_type + "_" + to_string(sid.module_num_);
    return sid;
}
SModuleID create_module_id(string module_type, unsigned int module_num){
    return create_module_id(module_type, "", module_num);
}
SModuleID create_module_id(){
    return create_module_id("UNKNOWN", "UNKNOWN", 0);
}
SModuleID create_sid_fromMsg(const ms_module_msgs::msg::ModuleID& sid_msg){
    SModuleID sid;
    sid = create_module_id(sid_msg.type, sid_msg.subtype, sid_msg.num);
    return sid;
}
ms_module_msgs::msg::ModuleID create_module_id_msg(const SModuleID& sid){
    ms_module_msgs::msg::ModuleID sid_msg;
    sid_msg.type = sid.module_type_;
    sid_msg.subtype = sid.module_subtype_;
    sid_msg.num = sid.module_num_;
    return sid_msg;
}
ms_module_msgs::msg::ModuleID create_module_id_msg(string module_type, string module_subtype, unsigned int module_num){
    assert(module_type != "" && "varible module_type can not empty when create_module_id");
    ms_module_msgs::msg::ModuleID sid_msg;
    sid_msg.type = module_type;
    sid_msg.subtype = module_subtype;
    sid_msg.num = module_num;
    return sid_msg;
}
ms_module_msgs::msg::ModuleID create_module_id_msg(string module_type, unsigned int module_num){
    return create_module_id_msg(module_type, "", module_num);
}
ms_module_msgs::msg::ModuleID create_module_id_msg(){
    return create_module_id_msg("UNKNOWN", "UNKNOWN", 0);
}


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** define connector
struct SConnectorID
{
    SModuleID module_id_;
    string connector_type_;
    string connector_subtype_;
    unsigned int connector_num_;
    string id_str_;
    string id_str_short_;
};
bool is_id_same(const SConnectorID& sid1, const SConnectorID& sid2){
    return(is_id_same(sid1.module_id_, sid2.module_id_)
        && sid1.connector_type_ == sid2.connector_type_
        && sid1.connector_subtype_ == sid2.connector_subtype_
        && sid1.connector_num_ == sid2.connector_num_);    
}
bool is_id_same(const ms_module_msgs::msg::ConnectorID& sid_msg1, const ms_module_msgs::msg::ConnectorID& sid_mgs2){
    return(is_id_same(sid_msg1.module_id, sid_mgs2.module_id)
        && sid_msg1.type == sid_mgs2.type
        && sid_msg1.subtype == sid_mgs2.subtype
        && sid_msg1.num == sid_mgs2.num);    
}
SConnectorID create_connector_id(string module_type, string module_subtype, unsigned int module_num,
        string connector_type, string connector_subtype, unsigned int connector_num)
{
    if(module_type == ""){
        printf("varible module_type can not empty when create_connector_id");
        module_type = "UNKONWN";
    }

    SConnectorID sid;
    sid.module_id_ = create_module_id(module_type, module_subtype, module_num);
    sid.connector_type_ = connector_type;
    sid.connector_subtype_ = connector_subtype;
    sid.connector_num_ = connector_num;

    sid.id_str_ = sid.module_id_.id_str_;
    if(sid.connector_type_ != ""){
        sid.id_str_ = sid.id_str_ + "_" + sid.connector_type_;
    }else{
        sid.id_str_ = sid.id_str_ + "_C";
    }
    if(sid.connector_subtype_ != ""){
        sid.id_str_ = sid.id_str_ + "_" + sid.connector_subtype_;
    }
    sid.id_str_ = sid.id_str_ + "_" + to_string(sid.connector_num_);
    sid.id_str_short_ = "C" + to_string(sid.connector_num_);
    return sid;
}
SConnectorID create_connector_id(string module_type, unsigned int module_num, unsigned int connector_num){
    return create_connector_id(module_type, "", module_num, "", "", connector_num);
}
SConnectorID create_connector_id(){
    return create_connector_id("UNKNOWN", "UNKNOWN", 0, "UNKNOWN", "UNKNOWN", 0);
}
SConnectorID create_sid_fromMsg(ms_module_msgs::msg::ConnectorID sid_msg){
    SConnectorID sid;
    sid = create_connector_id(sid_msg.module_id.type, sid_msg.module_id.subtype, sid_msg.module_id.num,
                    sid_msg.type, sid_msg.subtype, sid_msg.num);
    return sid;
}
ms_module_msgs::msg::ConnectorID create_connector_id_msg(string module_type, string module_subtype, unsigned int module_num,
        string connector_type, string connector_subtype, unsigned int connector_num)
{
    assert(module_type != "" && "varible module_type can not empty when create_connector_id");
    ms_module_msgs::msg::ConnectorID sid_msg;
    sid_msg.module_id = create_module_id_msg(module_type, module_subtype, module_num);
    sid_msg.type = connector_type;
    sid_msg.subtype = connector_subtype;
    sid_msg.num = connector_num;
    return sid_msg;
}
ms_module_msgs::msg::ConnectorID create_connector_id_msg(string module_type, unsigned int module_num, unsigned int connector_num){
    return create_connector_id_msg(module_type, "", module_num, "", "", connector_num);
}
ms_module_msgs::msg::ConnectorID create_connector_id_msg(){
    return create_connector_id_msg("UNKNOWN", "UNKNOWN", 0, "UNKNOWN", "UNKNOWN", 0);
}
ms_module_msgs::msg::ConnectorID create_connector_id_msg(SConnectorID sid){
    ms_module_msgs::msg::ConnectorID sid_msg;
    sid_msg = create_connector_id_msg(sid.module_id_.module_type_,
                            sid.module_id_.module_subtype_,
                            sid.module_id_.module_num_,
                            sid.connector_type_,
                            sid.connector_subtype_,
                            sid.connector_num_);
    return sid_msg;
}


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** define absorber
struct SAbsorberID
{
    SConnectorID connector_id_;
    string id_str_;
    string id_str_short_;
};
bool is_id_same(const SAbsorberID& sid1, const SAbsorberID& sid2){
    return is_id_same(sid1.connector_id_, sid2.connector_id_);
}
bool is_id_same(const ms_module_msgs::msg::AbsorberID& sid_msg1, const ms_module_msgs::msg::AbsorberID& sid_mgs2){
    return is_id_same(sid_msg1.connector_id, sid_mgs2.connector_id);    
}
SAbsorberID create_absorber_id(string module_type, string module_subtype, unsigned int module_num,
        string connector_type, string connector_subtype, unsigned int connector_num)
{
    if(module_type == ""){
        printf("varible module_type can not empty when create_absorber_id");
        module_type = "UNKONWN";
    }
    SAbsorberID sid;
    sid.connector_id_ = create_connector_id(module_type, module_subtype, module_num,
                                connector_type, connector_subtype, connector_num);

    sid.id_str_ = sid.connector_id_.id_str_ + "_" + "A";
    sid.id_str_short_ = sid.connector_id_.id_str_short_ + "_" + "A";
    return sid;
}
SAbsorberID create_absorber_id(string module_type, unsigned int module_num, unsigned int connector_num){
    return create_absorber_id(module_type, "", module_num, "", "", connector_num);
}
SAbsorberID create_absorber_id(){
    return create_absorber_id("UNKNOWN", "UNKNOWN", 0, "UNKNOWN", "UNKNOWN", 0);
}
SAbsorberID create_sid_fromMsg(ms_module_msgs::msg::AbsorberID sid_msg){
    SAbsorberID sid;
    sid = create_absorber_id(sid_msg.connector_id.module_id.type, sid_msg.connector_id.module_id.subtype, sid_msg.connector_id.module_id.num,
                    sid_msg.connector_id.type, sid_msg.connector_id.subtype, sid_msg.connector_id.num);
    return sid;
}
ms_module_msgs::msg::AbsorberID create_absorber_id_msg(string module_type, string module_subtype, unsigned int module_num,
        string connector_type, string connector_subtype, unsigned int connector_num)
{
    assert(module_type != "" && "varible module_type can not empty when create_absorber_id");
    ms_module_msgs::msg::AbsorberID sid_msg;
    sid_msg.connector_id.module_id = create_module_id_msg(module_type, module_subtype, module_num);
    sid_msg.connector_id.type = connector_type;
    sid_msg.connector_id.subtype = connector_subtype;
    sid_msg.connector_id.num = connector_num;
    return sid_msg;

    return sid_msg;
}
ms_module_msgs::msg::AbsorberID create_absorber_id_msg(string module_type, unsigned int module_num, unsigned int connector_num){
    return create_absorber_id_msg(module_type, "", module_num, "", "", connector_num);
}
ms_module_msgs::msg::AbsorberID create_absorber_id_msg(){
    return create_absorber_id_msg("UNKNOWN", "UNKNOWN", 0, "UNKNOWN", "UNKNOWN", 0);
}
ms_module_msgs::msg::AbsorberID create_absorber_id_msg(SAbsorberID sid){
    ms_module_msgs::msg::AbsorberID sid_msg;
    sid_msg = create_absorber_id_msg(sid.connector_id_.module_id_.module_type_,
                            sid.connector_id_.module_id_.module_subtype_,
                            sid.connector_id_.module_id_.module_num_,
                            sid.connector_id_.connector_type_,
                            sid.connector_id_.connector_subtype_,
                            sid.connector_id_.connector_num_);
    return sid_msg;
}


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** define joint
struct SJointID
{
    SModuleID module_id_;
    string joint_type_;
    string joint_subtype_;
    unsigned int joint_num_;
    string id_str_;
    string id_str_short_;
};
bool is_id_same(const SJointID& sid1, const SJointID& sid2){
    return(is_id_same(sid1.module_id_, sid2.module_id_)
        && sid1.joint_type_ == sid2.joint_type_
        && sid1.joint_subtype_ == sid2.joint_subtype_
        && sid1.joint_num_ == sid2.joint_num_);  
}
bool is_id_same(const ms_module_msgs::msg::JointID& sid_msg1, const ms_module_msgs::msg::JointID& sid_msg2){
    return(is_id_same(sid_msg1.module_id, sid_msg2.module_id)
        && sid_msg1.type == sid_msg2.type
        && sid_msg1.subtype == sid_msg2.subtype
        && sid_msg1.num == sid_msg2.num);  
}
SJointID create_joint_id(string module_type, string module_subtype, unsigned int module_num,
        string joint_type, string joint_subtype, unsigned int joint_num)
{
    if(module_type == ""){
        printf("varible module_type can not empty when create_joint_id");
        module_type = "UNKONWN";
    }
    SJointID sid;
    sid.module_id_ = create_module_id(module_type, module_subtype, module_num);
    sid.joint_type_ = joint_type;
    sid.joint_subtype_ = joint_subtype;
    sid.joint_num_ = joint_num;

    sid.id_str_ = sid.module_id_.id_str_;
    if(sid.joint_type_ != ""){
        sid.id_str_ = sid.id_str_ + "_" + sid.joint_type_;
    }else{
        sid.id_str_ = sid.id_str_ + "_J";
    }
    if(sid.joint_subtype_ != ""){
        sid.id_str_ = sid.id_str_ + "_" + sid.joint_subtype_;
    }
    sid.id_str_ = sid.id_str_ + "_" + to_string(sid.joint_num_);
    sid.id_str_short_ = "J" + to_string(sid.joint_num_);
    return sid;
}
SJointID create_joint_id(string module_type, unsigned int module_num, unsigned int joint_num){
    return create_joint_id(module_type, "", module_num, "", "", joint_num);
}
SJointID create_joint_id(){
    return create_joint_id("UNKNOWN", "UNKNOWN", 0, "UNKNOWN", "UNKNOWN", 0);
}
SJointID create_sid_fromMsg(const ms_module_msgs::msg::JointID& sid_msg){
    SJointID sid;
    sid = create_joint_id(sid_msg.module_id.type, sid_msg.module_id.subtype, sid_msg.module_id.num,
                    sid_msg.type, sid_msg.subtype, sid_msg.num);
    return sid;
}
ms_module_msgs::msg::JointID create_joint_id_msg(string module_type, string module_subtype, unsigned int module_num,
        string joint_type, string joint_subtype, unsigned int joint_num)
{
    assert(module_type != "" && "varible module_type can not empty when create_joint_id");
    ms_module_msgs::msg::JointID sid_msg;
    sid_msg.module_id = create_module_id_msg(module_type, module_subtype, module_num);
    sid_msg.type = joint_type;
    sid_msg.subtype = joint_subtype;
    sid_msg.num = joint_num;
    return sid_msg;
}
ms_module_msgs::msg::JointID create_joint_id_msg(string module_type, unsigned int module_num, unsigned int joint_num){
    return create_joint_id_msg(module_type, "", module_num, "", "", joint_num);
}
ms_module_msgs::msg::JointID create_joint_id_msg(){
    return create_joint_id_msg("UNKNOWN", "UNKNOWN", 0, "UNKNOWN", "UNKNOWN", 0);
}
ms_module_msgs::msg::JointID create_joint_id_msg(const SJointID& sid){
    ms_module_msgs::msg::JointID sid_msg;
    sid_msg = create_joint_id_msg(sid.module_id_.module_type_,
                            sid.module_id_.module_subtype_,
                            sid.module_id_.module_num_,
                            sid.joint_type_,
                            sid.joint_subtype_,
                            sid.joint_num_);
    return sid_msg;
}


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** joint
class CModule_Joint{
public:
    SJointID joint_sid_;
    double position_;
    double velocity_;
    double effort_;
    bool is_healthy_;
    string error_str_;
    vector<string> error_indexes_;
    vector<int64_t> error_values_;
    string info_str;
    CModule_Joint(){;}
    CModule_Joint(string module_type, string module_subtype, unsigned int module_num,
                string joint_type, string joint_subtype, unsigned int joint_num){
        init(module_type, module_subtype, module_num,
                joint_type, joint_subtype, joint_num);
    }
    CModule_Joint(const ms_module_msgs::msg::JointState& msg){
        set_joint_state(msg);
    }
    void init(string module_type, string module_subtype, unsigned int module_num,
                string joint_type, string joint_subtype, unsigned int joint_num)
    {
        joint_sid_ = create_joint_id(module_type, module_subtype, module_num,
                    joint_type, joint_subtype, joint_num);
        position_ = 0;
        velocity_ = 0;
        effort_ = 0;
        is_healthy_ = true;
        error_str_ = "";
        error_indexes_.clear();
        error_values_.clear();
        create_info_str();
    }
    void set_joint_state(const ms_module_msgs::msg::JointState& msg){
        joint_sid_ = create_sid_fromMsg(msg.joint_id);
        position_ = msg.position;
        velocity_ = msg.velocity;
        effort_ = msg.effort;
        is_healthy_ = msg.is_healthy;
        error_str_ = msg.error_str;
        error_indexes_ = msg.error_indexes;
        error_values_ = msg.error_values;
        create_info_str();
    }
    void set_position(double postion_in) { position_ = postion_in; create_info_str();}
    void set_velocity(double velocity_in) { velocity_ = velocity_in; create_info_str();}
    void set_effort(double effort_in) { effort_ = effort_in; create_info_str();}
    void set_healthy(bool healthy_in) { is_healthy_ = healthy_in; create_info_str();}
    ms_module_msgs::msg::JointState get_joint_state_msg(const std::shared_ptr<rclcpp::Node>& node){
        ms_module_msgs::msg::JointState msg;
        msg.header = std_msgs::msg::Header();
        msg.header.frame_id = joint_sid_.id_str_;
        msg.header.stamp = node->now();
        msg.joint_id = create_joint_id_msg(joint_sid_);
        msg.position = position_;
        msg.velocity = velocity_;
        msg.effort = effort_;
        msg.is_healthy = is_healthy_;
        msg.error_str = error_str_;
        msg.error_indexes = error_indexes_;
        msg.error_values = error_values_;
        return msg;
    }
    void create_info_str(){
        info_str = "joint: " + joint_sid_.id_str_short_ + ", ";
        is_healthy_? (info_str = info_str + "ok, ") : (info_str = info_str + "err, ");
        info_str = info_str + "pos:" + to_string(position_, 1) + ", "
                    + "vel:" + to_string(velocity_, 1) + ", "
                    + "eft:" + to_string(effort_, 1) + ", ";
    }
    bool is_this_my_duty(const ms_module_msgs::msg::JointCmd& msg){
        return is_id_same(joint_sid_, create_sid_fromMsg(msg.joint_id));
    }
};



// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** Connector
class CModule_Connector{
public:
    // ** ConnectorState
    SConnectorID connector_sid_;
    bool is_locked_;
    bool is_healthy_;
    string error_str_;
    vector<string> error_indexes_;
    vector<int64_t> error_values_;
    string info_str;
    CModule_Connector(){;}
    CModule_Connector(string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        init(module_type, module_subtype, module_num,
                connector_type, connector_subtype, connector_num);
    }
    CModule_Connector(const ms_module_msgs::msg::ConnectorState& msg){
        set_connector_state(msg);
    }
    void init(string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        connector_sid_ = create_connector_id(module_type, module_subtype, module_num,
                    connector_type, connector_subtype, connector_num);
        is_locked_ = false;
        is_healthy_ = true;
        error_str_ = "";
        error_indexes_.clear();
        error_values_.clear();
    }
    void set_connector_state(const ms_module_msgs::msg::ConnectorState& msg){
        connector_sid_ = create_sid_fromMsg(msg.connector_id);
        is_locked_ = msg.is_locked;
        is_healthy_ = msg.is_healthy;
        error_str_ = msg.error_str;
        error_indexes_ = msg.error_indexes;
        error_values_ = msg.error_values;
        create_info_str();
    }
    void set_lock() { is_locked_ = true; create_info_str();}
    void set_unlock() { is_locked_ = false; create_info_str();}
    void set_healthy(bool healthy_in) { is_healthy_ = healthy_in; create_info_str();}
    ms_module_msgs::msg::ConnectorState get_connector_state_msg(const std::shared_ptr<rclcpp::Node>& node){
        ms_module_msgs::msg::ConnectorState msg;
        msg.header = std_msgs::msg::Header();
        msg.header.frame_id = connector_sid_.id_str_;
        msg.header.stamp = node->now();
        msg.connector_id = create_connector_id_msg(connector_sid_);
        msg.is_locked = is_locked_;
        msg.is_healthy = is_healthy_;
        msg.error_str = error_str_;
        msg.error_indexes = error_indexes_;
        msg.error_values = error_values_;
        return msg;
    }
    void create_info_str(){
        info_str = "connector: " + connector_sid_.id_str_short_ + ", ";
        is_healthy_? (info_str = info_str + "ok, ") : (info_str = info_str + "err, ");
        is_locked_? (info_str = info_str + "locked, ") : (info_str = info_str + "unlocked, ");
    }
    bool is_this_my_duty(const ms_module_msgs::msg::ConnectCmd& msg){
        return is_id_same(connector_sid_, create_sid_fromMsg(msg.connector_id));
    }
};



// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** Connector Absorber
class CModule_Connector_Absorber{
public:
    // ** ConnectorState
    SAbsorberID absorber_sid_;
    bool is_locked_;
    bool is_healthy_;
    string error_str_;
    vector<string> error_indexes_;
    vector<int64_t> error_values_;
    string info_str;
    CModule_Connector_Absorber(){;}
    CModule_Connector_Absorber(string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        init(module_type, module_subtype, module_num,
                connector_type, connector_subtype, connector_num);
    }
    CModule_Connector_Absorber(const ms_module_msgs::msg::AbsorberState& msg){
        set_absorber_state(msg);
    }
    void init(string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        absorber_sid_ = create_absorber_id(module_type, module_subtype, module_num,
                    connector_type, connector_subtype, connector_num);
        is_locked_ = false;
        is_healthy_ = true;
        error_str_ = "";
        error_indexes_.clear();
        error_values_.clear();
    }
    void set_absorber_state(const ms_module_msgs::msg::AbsorberState& msg){
        absorber_sid_ = create_sid_fromMsg(msg.absorber_id);
        is_locked_ = msg.is_locked;
        is_healthy_ = msg.is_healthy;
        error_str_ = msg.error_str;
        error_indexes_ = msg.error_indexes;
        error_values_ = msg.error_values;
        create_info_str();
    }
    void set_lock() { is_locked_ = true; create_info_str();}
    void set_unlock() { is_locked_ = false; create_info_str();}
    void set_healthy(bool healthy_in) { is_healthy_ = healthy_in; create_info_str();}
    ms_module_msgs::msg::AbsorberState get_absorber_state_msg(const std::shared_ptr<rclcpp::Node>& node){
        ms_module_msgs::msg::AbsorberState msg;
        msg.header = std_msgs::msg::Header();
        msg.header.frame_id = absorber_sid_.id_str_;
        msg.header.stamp = node->now();
        msg.absorber_id = create_absorber_id_msg(absorber_sid_);
        msg.is_locked = is_locked_;
        msg.is_healthy = is_healthy_;
        msg.error_str = error_str_;
        msg.error_indexes = error_indexes_;
        msg.error_values = error_values_;
        return msg;
    }
    void create_info_str(){
        info_str = "absorber: " + absorber_sid_.id_str_short_ + ", ";
        is_healthy_? (info_str = info_str + "ok, ") : (info_str = info_str + "err, ");
        is_locked_? (info_str = info_str + "locked, ") : (info_str = info_str + "unlocked, ");
    }
    bool is_this_my_duty(const ms_module_msgs::msg::AbsorberCmd& msg){
        return is_id_same(absorber_sid_, create_sid_fromMsg(msg.absorber_id));
    }
};



// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** create joint commands
class CModule_Joint_CMD{
public:
    SModuleID publisher_sid_;
    SJointID joint_sid_;
    int8_t control_type_;
    /* which following cmd acts
    # 0 position control only
    # 1 velocity control only
    # 2 effort control only
    # 3 position with max velocity of inputed velocity
    # 4 position with max effort of inputed effort
    # 5 position with max velocity and max effort of input
    # 6-9 vacancy
    # 10 reset_motor
    # 11 clear_error
    # 12 torque_enable
    # 13 control_enable
    # 14 lock */
    double position_target_;
    double velocity_target_;
    double effort_target_;
    bool reset_motor_;
    bool clear_error_;
    bool torque_enable_;
    bool control_enable_;
    bool lock_;
    string info_str;
    CModule_Joint_CMD(){;}
    CModule_Joint_CMD(string publisher_type, string publisher_subtype, unsigned int publisher_num,
                string module_type, string module_subtype, unsigned int module_num,
                string joint_type, string joint_subtype, unsigned int joint_num){
        init(publisher_type, publisher_subtype, publisher_num,
                module_type, module_subtype, module_num,
                joint_type, joint_subtype, joint_num);
    }
    CModule_Joint_CMD(SModuleID publisher_id,
                string module_type, string module_subtype, unsigned int module_num,
                string joint_type, string joint_subtype, unsigned int joint_num){
        init(publisher_id.module_type_, publisher_id.module_subtype_, publisher_id.module_num_,
                module_type, module_subtype, module_num,
                joint_type, joint_subtype, joint_num);
    }
    CModule_Joint_CMD(SModuleID publisher_id, SJointID joint_id){
        init(publisher_id.module_type_, publisher_id.module_subtype_, publisher_id.module_num_,
                joint_id.module_id_.module_type_, joint_id.module_id_.module_subtype_, joint_id.module_id_.module_num_,
                joint_id.joint_type_, joint_id.joint_subtype_, joint_id.joint_num_);
    }
    void init(string publisher_type, string publisher_subtype, unsigned int publisher_num,
                string module_type, string module_subtype, unsigned int module_num,
                string joint_type, string joint_subtype, unsigned int joint_num){
        publisher_sid_ = create_module_id(publisher_type, publisher_subtype, publisher_num);
        joint_sid_ = create_joint_id(module_type, module_subtype, module_num,
                    joint_type, joint_subtype, joint_num);
        position_target_ = 0;
        velocity_target_ = 0;
        effort_target_ = 0;
        control_type_ = 0;
        reset_motor_ = false;
        clear_error_ = false;
        torque_enable_ = false;
        control_enable_ = false;
        lock_ = false;
        create_info_str();
    }
    void set_joint_cmd(const ms_module_msgs::msg::JointCmd& msg){
        publisher_sid_ = create_sid_fromMsg(msg.publisher_id);
        joint_sid_ = create_sid_fromMsg(msg.joint_id);
        position_target_ = msg.position;
        velocity_target_ = msg.velocity;
        effort_target_ = msg.effort;
        control_type_ = msg.control_type;
        reset_motor_ = msg.reset_motor;
        clear_error_ = msg.clear_error;
        torque_enable_ = msg.torque_enable;
        control_enable_ = msg.control_enable;
        lock_ = msg.lock;
    }
    ms_module_msgs::msg::JointCmd create_joint_cmd_msg(const std::shared_ptr<rclcpp::Node>& node,
                    double position, double velocity, double effort, 
                    int8_t control_type, bool reset_motor, bool clear_error, bool torque_enable, bool control_enable, bool lock){
        ms_module_msgs::msg::JointCmd cmd;
        cmd.header = std_msgs::msg::Header();
        cmd.header.frame_id = joint_sid_.id_str_;
        cmd.header.stamp = node->now();
        cmd.publisher_id = create_module_id_msg(publisher_sid_);
        cmd.joint_id = create_joint_id_msg(joint_sid_);
        cmd.position = position;
        cmd.velocity = velocity;
        cmd.effort = effort;
        cmd.control_type = control_type;
        cmd.reset_motor = reset_motor;
        cmd.clear_error = clear_error;
        cmd.torque_enable = torque_enable;
        cmd.control_enable = control_enable;
        cmd.lock = lock;
        // save date for easy access
        set_joint_cmd(cmd);
        return cmd;
    }
    ms_module_msgs::msg::JointCmd create_position_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, double position_target){
        return create_joint_cmd_msg(node, position_target, 0, 0, 0, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_velocity_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, double velocity_target){
        return create_joint_cmd_msg(node, 0, velocity_target, 0, 1, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_effort_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, double effort_target){
        return create_joint_cmd_msg(node, 0, 0, effort_target, 2, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_position_with_max_velocity_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, double position_target, double velocity_limit){
        return create_joint_cmd_msg(node, position_target, velocity_limit, 0, 3, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_position_with_max_effort_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, double position_target, double effort_limit){
        return create_joint_cmd_msg(node, position_target, 0, effort_limit, 4, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_position_with_max_velcoty_and_effort_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, double position_target, double velocity_limit, double effort_limit){
        return create_joint_cmd_msg(node, position_target, velocity_limit, effort_limit, 5, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_reset_motor_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 10, true, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_clear_error_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 11, false, true, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_torque_enable_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 12, false, false, true, false, false);
    }
    ms_module_msgs::msg::JointCmd create_torque_disable_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 12, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_control_enable_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 13, false, false, false, true, false);
    }
    ms_module_msgs::msg::JointCmd create_control_disable_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 13, false, false, false, false, false);
    }
    ms_module_msgs::msg::JointCmd create_control_lock_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 14, false, false, false, false, true);
    }
    ms_module_msgs::msg::JointCmd create_control_unlock_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_joint_cmd_msg(node, 0, 0, 0, 14, false, false, false, false, false);
    }
    void create_info_str(){
        info_str = "leader:" + publisher_sid_.id_str_short_ + "asks " + "joint: " + joint_sid_.id_str_short_ + ", ";
        info_str = info_str + "pos:" + to_string(position_target_, 1) + ", "
                    + "vel:" + to_string(velocity_target_, 1) + ", "
                    + "eft:" + to_string(effort_target_, 1) + ", ";
        info_str = info_str + "\n";
    }
};



// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** create connector commands
class CModule_Connect_CMD{
public:
    SModuleID publisher_sid_;
    SConnectorID connector_sid_;
    bool lock_or_unlock_;
    string info_str;
    CModule_Connect_CMD(){;}
    CModule_Connect_CMD(string publisher_type, string publisher_subtype, unsigned int publisher_num,
                string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        init(publisher_type, publisher_subtype, publisher_num, 
                module_type, module_subtype, module_num,
                connector_type, connector_subtype, connector_num);
    }
    CModule_Connect_CMD(SModuleID publisher_id, 
                string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        init(publisher_id.module_type_, publisher_id.module_subtype_, publisher_id.module_num_,
                module_type, module_subtype, module_num,
                connector_type, connector_subtype, connector_num);
    }
    CModule_Connect_CMD(SModuleID publisher_id, SConnectorID connector_id){
        init(publisher_id.module_type_, publisher_id.module_subtype_, publisher_id.module_num_,
                connector_id.module_id_.module_type_, connector_id.module_id_.module_subtype_, connector_id.module_id_.module_num_,
                connector_id.connector_type_, connector_id.connector_subtype_, connector_id.connector_num_);
    }
    void init(string publisher_type, string publisher_subtype, unsigned int publisher_num,
                string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        publisher_sid_ = create_module_id(publisher_type, publisher_subtype, publisher_num);
        connector_sid_ = create_connector_id(module_type, module_subtype, module_num,
                    connector_type, connector_subtype, connector_num);
        lock_or_unlock_ = false;
        create_info_str();
    }

    void set_connect_cmd(ms_module_msgs::msg::ConnectCmd msg){
        publisher_sid_ = create_sid_fromMsg(msg.publisher_id);
        connector_sid_ = create_sid_fromMsg(msg.connector_id);
        lock_or_unlock_ = msg.lock_or_unlock;
        create_info_str();
    }
    ms_module_msgs::msg::ConnectCmd create_connector_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, bool to_connect){
        ms_module_msgs::msg::ConnectCmd cmd;
        cmd.header = std_msgs::msg::Header();
        cmd.header.frame_id = connector_sid_.id_str_;
        cmd.header.stamp = node->now();
        cmd.publisher_id = create_module_id_msg(publisher_sid_);
        cmd.connector_id = create_connector_id_msg(connector_sid_);
        cmd.lock_or_unlock = to_connect;
        // save date for easy access
        set_connect_cmd(cmd);
        return cmd;
    }
    ms_module_msgs::msg::ConnectCmd create_connect_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_connector_cmd_msg(node, true);
    }
    ms_module_msgs::msg::ConnectCmd create_disconnect_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_connector_cmd_msg(node, false);
    }
    void create_info_str(){
        info_str = "leader:" + publisher_sid_.id_str_short_ + "asks " + "connector: " + connector_sid_.id_str_short_ + ", ";
        lock_or_unlock_? (info_str = info_str + "to_lock, ") : (info_str = info_str + "-, ");
        info_str = info_str + "\n";
    }
};


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** create connector absorber commands
class CModule_Connector_Absorber_CMD{
public:
    SModuleID publisher_sid_;
    SAbsorberID absorber_sid_;
    bool lock_or_unlock_;
    string info_str;
    CModule_Connector_Absorber_CMD(){;}
    CModule_Connector_Absorber_CMD(string publisher_type, string publisher_subtype, unsigned int publisher_num,
                string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        init(publisher_type, publisher_subtype, publisher_num, 
                module_type, module_subtype, module_num,
                connector_type, connector_subtype, connector_num);
    }
    CModule_Connector_Absorber_CMD(SModuleID publisher_id, 
                string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        init(publisher_id.module_type_, publisher_id.module_subtype_, publisher_id.module_num_,
                module_type, module_subtype, module_num,
                connector_type, connector_subtype, connector_num);
    }
    CModule_Connector_Absorber_CMD(SModuleID publisher_id, SConnectorID connector_id){
        init(publisher_id.module_type_, publisher_id.module_subtype_, publisher_id.module_num_,
                connector_id.module_id_.module_type_, connector_id.module_id_.module_subtype_, connector_id.module_id_.module_num_,
                connector_id.connector_type_, connector_id.connector_subtype_, connector_id.connector_num_);
    }
    void init(string publisher_type, string publisher_subtype, unsigned int publisher_num,
                string module_type, string module_subtype, unsigned int module_num,
                string connector_type, string connector_subtype, unsigned int connector_num){
        publisher_sid_ = create_module_id(publisher_type, publisher_subtype, publisher_num);
        absorber_sid_ = create_absorber_id(module_type, module_subtype, module_num,
                    connector_type, connector_subtype, connector_num);
        lock_or_unlock_ = false;
        create_info_str();
    }

    void set_rigidify_cmd(ms_module_msgs::msg::AbsorberCmd msg){
        publisher_sid_ = create_sid_fromMsg(msg.publisher_id);
        absorber_sid_ = create_sid_fromMsg(msg.absorber_id);
        lock_or_unlock_ = msg.lock_or_unlock;
        create_info_str();
    }
    ms_module_msgs::msg::AbsorberCmd create_absorber_cmd_msg(const std::shared_ptr<rclcpp::Node>& node, bool to_rigidify){
        ms_module_msgs::msg::AbsorberCmd cmd;
        cmd.header = std_msgs::msg::Header();
        cmd.header.frame_id = absorber_sid_.id_str_;
        cmd.header.stamp = node->now();
        cmd.publisher_id = create_module_id_msg(publisher_sid_);
        cmd.absorber_id = create_absorber_id_msg(absorber_sid_);
        cmd.lock_or_unlock = to_rigidify;
        // save date for easy access
        set_rigidify_cmd(cmd);
        return cmd;
    }
    ms_module_msgs::msg::AbsorberCmd create_rigidify_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_absorber_cmd_msg(node, true);
    }
    ms_module_msgs::msg::AbsorberCmd create_flex_cmd_msg(const std::shared_ptr<rclcpp::Node>& node){
        return create_absorber_cmd_msg(node, false);
    }
    void create_info_str(){
        info_str = "leader:" + publisher_sid_.id_str_short_ + "asks " + "absorber: " + absorber_sid_.id_str_short_ + ", ";
        lock_or_unlock_? (info_str = info_str + "to_lock, ") : (info_str = info_str + "-, ");
        info_str = info_str + "\n";
    }
};


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** convient func
// *** module ids

// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** convient func
// *** joints ids

// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** convient func
// *** connector ids


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** connection
class CModule_Connection{
    //TODO
};


// ******************************************************************
// ******************************************************************
// ******************************************************************
// *** module
class CModule{
public:
    SModuleID module_sid_;
    vector<CModule_Joint> joint_list_;
    vector<CModule_Connector> connector_list_;
    vector<CModule_Connector_Absorber> absorber_list_;
    vector<CModule_Connection> connection_list_;
    geometry_msgs::msg::Pose robot_pose_;
    sensor_msgs::msg::BatteryState battery_state_;
    bool is_healthy_;
    string error_str_;
    vector<string> error_indexes_;
    vector<int64_t> error_values_;
    string info_str;
    //TODO
};


# endif //MS_MODULE_MSGS_ULIL_H_
