// ***
// *** tf utilies of ROS2
// *** Auther: xixun wang
// *** Purpose: swap the data between tfs and ros2 msg
// *** update 20240123: create ToTF and ToTFMsg
// *** update 20240211: create createQuaternionMsgFromRollPitchYaw which is used in tf1
// *** update 20240326: add manul time in ToTFMsg
// *** update 20240326: create getTFTime_now
// *** 

#ifndef TF_UTIL_H_
#define TF_UTIL_H_

// *** std include
#include <stdio.h>
#include <iostream>
using namespace std;

// *** ros2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI 3.1415926535

// *** time
builtin_interfaces::msg::Time getTimeMsg();
builtin_interfaces::msg::Time getTimeMsg(const std::shared_ptr<rclcpp::Node>& node);
builtin_interfaces::msg::Time getTimeMsg_fromTime(double time);
builtin_interfaces::msg::Time getTimeMsg_fromTFTime(const tf2::TimePoint& time);
double getTime();
double getTime(const std::shared_ptr<rclcpp::Node>& node);
double getTime_fromMsg(const builtin_interfaces::msg::Time& time);
double getTime_fromTFTime(const tf2::TimePoint& time);
tf2::TimePoint getTFTime_now();
tf2::TimePoint getTFTime_now(const std::shared_ptr<rclcpp::Node>& node);
tf2::TimePoint getTFTime_fromTimeMsg(const builtin_interfaces::msg::Time& time);
tf2::TimePoint getTFTime_fromTime(double time);
// *** initial
vector<double> zero_vector();
geometry_msgs::msg::Pose zero_pose();
geometry_msgs::msg::PoseStamped zero_pose_stamped();
tf2::Transform zero_tf();
// *** vec <--> number
vector<double> positionRPY2vector(double x, double y, double z, double R, double P, double Y);
// *** pose <--> tf
tf2::Transform pose2tf(const geometry_msgs::msg::Pose& pose_in);
geometry_msgs::msg::Pose tf2pose(const tf2::Transform& tf_in);
// *** vec <--> tf
tf2::Transform vector2tf(const vector<double>& vector_in);
vector<double> tf2vector(const tf2::Transform& tf_in);
// *** vec <--> pose
geometry_msgs::msg::Pose vector2pose(const vector<double>& vector_in);
vector<double> pose2vector(const geometry_msgs::msg::Pose& pose_in);
// *** number <--> pose
void pose2positionRPY(const geometry_msgs::msg::Pose& pose, double& x, double& y, double& z, double& R, double& P, double& Y);
void positionRPY2pose(double x, double y, double z, double R, double P, double Y, geometry_msgs::msg::Pose& pose);
geometry_msgs::msg::Pose positionRPY2pose(double x, double y, double z, double R, double P, double Y);
// *** orientation <--> quaternion
geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(double R, double P, double Y);
void quaternion2RPY(double x, double y, double z, double w, double& R, double& P, double& Y);
void quaternionMsg2RPY(const geometry_msgs::msg::Quaternion& q_msg, double& R, double& P, double& Y);
void quaternionTF2RPY(const tf2::Quaternion& q, double& R, double& P, double& Y);
// *** number <--> tf
void tf2positionRPY(const tf2::Transform& tf, double& x, double& y, double& z, double& R, double& P, double& Y);
void positionRPY2tf(double x, double y, double z, double R, double P, double Y, tf2::Transform& tf);
tf2::Transform positionRPY2tf(double x, double y, double z, double R, double P, double Y);
// *** -> tf_msg for publishing tf
geometry_msgs::msg::TransformStamped pose2tf_msg(const geometry_msgs::msg::Pose& pose_in, double time, string parent_frame, string child_frame);
geometry_msgs::msg::TransformStamped pose2tf_msg(const geometry_msgs::msg::Pose& pose_in, string parent_frame, string child_frame)
{ return pose2tf_msg(pose_in, getTime(), parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped tf2tf_msg(const tf2::Transform& tf_in, double time, string parent_frame, string child_frame);
geometry_msgs::msg::TransformStamped tf2tf_msg(const tf2::Transform& tf_in, string parent_frame, string child_frame)
{ return tf2tf_msg(tf_in, getTime(), parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped vector2tf_msg(const vector<double>& vector_in, double time, string parent_frame, string child_frame);
geometry_msgs::msg::TransformStamped vector2tf_msg(const vector<double>& vector_in, string parent_frame, string child_frame)
{ return vector2tf_msg(vector_in, getTime(), parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped positionRPY2tf_msg(double x, double y, double z, double R, double P, double Y, double time, string parent_frame, string child_frame);
geometry_msgs::msg::TransformStamped positionRPY2tf_msg(double x, double y, double z, double R, double P, double Y, string parent_frame, string child_frame)
{ return positionRPY2tf_msg(x, y, z, R, P, Y, getTime(), parent_frame, child_frame); }
geometry_msgs::msg::Pose tf_msg2pose(const geometry_msgs::msg::TransformStamped& tf_msg);
tf2::Transform tf_msg2tf(const geometry_msgs::msg::TransformStamped& tf_msg);
vector<double> tf_msg2vector(const geometry_msgs::msg::TransformStamped& tf_msg);
void tf_msg2positionRPY(const geometry_msgs::msg::TransformStamped& tf_msg, double& x, double& y, double& z, double& R, double& P, double& Y);
// *** wrap
tf2::Transform ToTF(double x, double y, double z, double R, double P, double Y)
{ return positionRPY2tf(x,y,z,R,P,Y); }
tf2::Transform ToTF(const vector<double>& vector_in)
{ return vector2tf(vector_in); }
tf2::Transform ToTF(const geometry_msgs::msg::Pose& pose_in)
{ return pose2tf(pose_in); }
tf2::Transform ToTF(const geometry_msgs::msg::TransformStamped& tf_msg)
{ return tf_msg2tf(tf_msg); }
geometry_msgs::msg::TransformStamped ToTFMsg(double x, double y, double z, double R, double P, double Y, double time, string parent_frame, string child_frame)
{ return positionRPY2tf_msg(x,y,z,R,P,Y, time, parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped ToTFMsg(double x, double y, double z, double R, double P, double Y, string parent_frame, string child_frame)
{ return positionRPY2tf_msg(x,y,z,R,P,Y, parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped ToTFMsg(const vector<double>& vector_in, double time, string parent_frame, string child_frame)
{ return vector2tf_msg(vector_in, time, parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped ToTFMsg(const vector<double>& vector_in, string parent_frame, string child_frame)
{ return vector2tf_msg(vector_in, parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped ToTFMsg(const geometry_msgs::msg::Pose& pose_in, double time, string parent_frame, string child_frame)
{ return pose2tf_msg(pose_in, time, parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped ToTFMsg(const geometry_msgs::msg::Pose& pose_in, string parent_frame, string child_frame)
{ return pose2tf_msg(pose_in, parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped ToTFMsg(const tf2::Transform& tf_in, double time, string parent_frame, string child_frame)
{ return tf2tf_msg(tf_in, time, parent_frame, child_frame); }
geometry_msgs::msg::TransformStamped ToTFMsg(const tf2::Transform& tf_in, string parent_frame, string child_frame)
{ return tf2tf_msg(tf_in, parent_frame, child_frame); }
geometry_msgs::msg::Pose ToPose(double x, double y, double z, double R, double P, double Y)
{ return positionRPY2pose(x,y,z,R,P,Y); }
geometry_msgs::msg::Pose ToPose(const vector<double>& vector_in)
{ return vector2pose(vector_in); }
geometry_msgs::msg::Pose ToPose(const tf2::Transform& tf_in)
{ return tf2pose(tf_in); }
geometry_msgs::msg::Pose ToPose(const geometry_msgs::msg::TransformStamped& tf_msg)
{ return tf_msg2pose(tf_msg); }
vector<double> ToVector(double x, double y, double z, double R, double P, double Y)
{ return positionRPY2vector(x, y, z, R, P, Y); }
vector<double> ToVector(const geometry_msgs::msg::Pose& pose_in)
{ return pose2vector(pose_in); }
vector<double> ToVector(const tf2::Transform& tf_in)
{ return tf2vector(tf_in); }
vector<double> ToVector(const geometry_msgs::msg::TransformStamped& tf_msg)
{ return tf_msg2vector(tf_msg); }
void ToPositionRPY(const geometry_msgs::msg::Pose& pose, double& x, double& y, double& z, double& R, double& P, double& Y)
{ pose2positionRPY(pose, x, y, z, R, P, Y); }
void ToPositionRPY(const geometry_msgs::msg::TransformStamped& tf_msg, double& x, double& y, double& z, double& R, double& P, double& Y)
{ tf_msg2positionRPY(tf_msg, x, y, z, R, P, Y); }
void ToPositionRPY(const tf2::Transform& tf, double& x, double& y, double& z, double& R, double& P, double& Y)
{ tf2positionRPY(tf, x, y, z, R, P, Y); }
geometry_msgs::msg::Quaternion ToQuaternionMsg(double R, double P, double Y)
{ return createQuaternionMsgFromRollPitchYaw(R,P,Y);}
void ToRPY(double x, double y, double z, double w, double& R, double& P, double& Y)
{ quaternion2RPY(x, y, z, w, R, P, Y); }
void ToRPY(const geometry_msgs::msg::Quaternion& q_msg, double& R, double& P, double& Y)
{ quaternionMsg2RPY(q_msg, R, P, Y); }
void ToRPY(const tf2::Quaternion& q, double& R, double& P, double& Y)
{ quaternionTF2RPY(q, R, P, Y); }
// *** mean
geometry_msgs::msg::Pose mean(const vector<geometry_msgs::msg::Pose>& poses);
tf2::Transform mean(const vector<tf2::Transform>& tfs);
vector<double> mean(const vector< vector<double> >& vectors);
// *** show
void show_vector(const vector<double>& vector_in);
void show_pose(const geometry_msgs::msg::Pose& pose_in);
void show_tf(const tf2::Transform& tf_in);
double pi2pi(double Angle);
double pi_2_2pi(double angle);
double angles_diff(double angle1, double angle2);



vector<double> zero_vector()
{
    vector<double> vector_in(6);
    vector_in[0] = 0;
    vector_in[1] = 0;
    vector_in[2] = 0;
    vector_in[3] = 0;
    vector_in[4] = 0;
    vector_in[5] = 0;
    return vector_in;
}
geometry_msgs::msg::Pose zero_pose()
{
    return vector2pose(zero_vector());
}
geometry_msgs::msg::PoseStamped zero_pose_stamped()
{
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = vector2pose(zero_vector());
    return pose;
}
tf2::Transform zero_tf()
{
    return tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.0, 0.0, 0.0));
}
vector<double> positionRPY2vector(double x, double y, double z, double R, double P, double Y)
{
    vector<double> vector_in(6);
    vector_in[0] = x;
    vector_in[1] = y;
    vector_in[2] = z;
    vector_in[3] = R;
    vector_in[4] = P;
    vector_in[5] = Y;
    return vector_in;
}

tf2::Transform pose2tf(const geometry_msgs::msg::Pose& pose_in)
{
    tf2::Transform pose_tf;
    tf2::fromMsg(pose_in, pose_tf);
    return pose_tf;
}
geometry_msgs::msg::Pose tf2pose(const tf2::Transform& tf_in)
{
    geometry_msgs::msg::Pose pose_msg;
    tf2::Vector3 position_vector =  tf_in.getOrigin();
    pose_msg.position.x = position_vector.x();
    pose_msg.position.y = position_vector.y();
    pose_msg.position.z = position_vector.z();
    tf2::Quaternion q = tf_in.getRotation();
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    return pose_msg;
}

tf2::Transform vector2tf(const vector<double>& vector_in)
{
    if(vector_in.size() != 6)
    {
        printf("Error in vector2tf, the size of input vector_in is not 6, return empty");
        return tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.0, 0.0, 0.0));
    }
    double x = vector_in[0];
    double y = vector_in[1];
    double z = vector_in[2];
    double R = vector_in[3];
    double P = vector_in[4];
    double Y = vector_in[5];
    
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, z));
    tf2::Quaternion quat;
    quat.setRPY(R, P, Y);
    transform.setRotation(quat);

    return transform;
}
vector<double> tf2vector(const tf2::Transform& tf_in)
{
    double x,y,z,R, P, Y;
    tf2::Matrix3x3(tf_in.getRotation()).getRPY(R, P, Y);
    tf2::Vector3 position_vector =  tf_in.getOrigin();
    x = position_vector.x();
    y = position_vector.y();
    z = position_vector.z();
    
    vector<double> pose_vector(6);
    pose_vector[0] = x;
    pose_vector[1] = y;
    pose_vector[2] = z;
    pose_vector[3] = R;
    pose_vector[4] = P;
    pose_vector[5] = Y;

    return pose_vector;
}
geometry_msgs::msg::Pose vector2pose(const vector<double>& vector_in)
{
    if(vector_in.size() != 6)
    {
        printf("Error in vector2pose, the size of input vector_in is not 6, return empty");
        return zero_pose();
    }
    
    tf2::Transform transform = vector2tf(vector_in);
    return tf2pose(transform);
}
vector<double> pose2vector(const geometry_msgs::msg::Pose& pose_in)
{
    double x = pose_in.position.x;
    double y = pose_in.position.y;
    double z = pose_in.position.z;
    double R, P, Y;
    tf2::Transform transform = pose2tf(pose_in);
    tf2::Matrix3x3(transform.getRotation()).getRPY(R, P, Y);
    
    vector<double> pose_vector(6);
    pose_vector[0] = x;
    pose_vector[1] = y;
    pose_vector[2] = z;
    pose_vector[3] = R;
    pose_vector[4] = P;
    pose_vector[5] = Y;

    return pose_vector;
}
void pose2positionRPY(const geometry_msgs::msg::Pose& pose, double& x, double& y, double& z, double& R, double& P, double& Y)
{
    geometry_msgs::msg::Point position = pose.position;
    x = position.x;
    y = position.y;
    z = position.z;
    geometry_msgs::msg::Quaternion orientation = pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3(q).getRPY(R, P, Y);
}
void positionRPY2pose(double x, double y, double z, double R, double P, double Y, geometry_msgs::msg::Pose& pose)
{
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    geometry_msgs::msg::Quaternion q = createQuaternionMsgFromRollPitchYaw(R, P, Y);
    pose.orientation = q;
}
geometry_msgs::msg::Pose positionRPY2pose(double x, double y, double z, double R, double P, double Y)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    geometry_msgs::msg::Quaternion q = createQuaternionMsgFromRollPitchYaw(R, P, Y);
    pose.orientation = q;
    return pose;
}
geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(double R, double P, double Y)
{
    tf2::Quaternion q;
    q.setRPY(R, P, Y);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
}
void quaternion2RPY(double x, double y, double z, double w, double& R, double& P, double& Y)
{
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3(q).getRPY(R, P, Y);
}
void quaternionMsg2RPY(const geometry_msgs::msg::Quaternion& q_msg, double& R, double& P, double& Y)
{
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    tf2::Matrix3x3(q).getRPY(R, P, Y);
}
void quaternionTF2RPY(const tf2::Quaternion& q, double& R, double& P, double& Y)
{
    tf2::Matrix3x3(q).getRPY(R, P, Y);
}
void tf2positionRPY(const tf2::Transform& tf, double& x, double& y, double& z, double& R, double& P, double& Y)
{
    geometry_msgs::msg::Pose pose = tf2pose(tf);
    pose2positionRPY(pose, x, y, z, R, P, Y);
}
void positionRPY2tf(double x, double y, double z, double R, double P, double Y, tf2::Transform& tf)
{
    geometry_msgs::msg::Pose pose = positionRPY2pose(x, y, z, R, P, Y);
    tf = pose2tf(pose);
}
tf2::Transform positionRPY2tf(double x, double y, double z, double R, double P, double Y)
{
    geometry_msgs::msg::Pose pose = positionRPY2pose(x, y, z, R, P, Y);
    return pose2tf(pose);
}
geometry_msgs::msg::TransformStamped pose2tf_msg(const geometry_msgs::msg::Pose& pose_in, double time, string parent_frame, string child_frame)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = getTimeMsg_fromTime(time);
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;
    t.transform.translation.x = pose_in.position.x;
    t.transform.translation.y = pose_in.position.y;
    t.transform.translation.z = pose_in.position.z;
    t.transform.rotation.x = pose_in.orientation.x;
    t.transform.rotation.y = pose_in.orientation.y;
    t.transform.rotation.z = pose_in.orientation.z;
    t.transform.rotation.w = pose_in.orientation.w;
    return t;
}
geometry_msgs::msg::TransformStamped tf2tf_msg(const tf2::Transform& tf_in, double time, string parent_frame, string child_frame)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = getTimeMsg_fromTime(time);
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;
    tf2::Vector3 position_vector =  tf_in.getOrigin();
    t.transform.translation.x = position_vector.x();
    t.transform.translation.y = position_vector.y();
    t.transform.translation.z = position_vector.z();
    tf2::Quaternion q = tf_in.getRotation();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    return t;
}
geometry_msgs::msg::TransformStamped vector2tf_msg(const vector<double>& vector_in, double time, string parent_frame, string child_frame)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = getTimeMsg_fromTime(time);
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;

    t.transform.translation.x = vector_in[0];
    t.transform.translation.y = vector_in[1];
    t.transform.translation.z = vector_in[2];

    tf2::Quaternion q;
    q.setRPY(vector_in[3], vector_in[4], vector_in[5]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
}
geometry_msgs::msg::TransformStamped positionRPY2tf_msg(double x, double y, double z, double R, double P, double Y, double time, string parent_frame, string child_frame)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = getTimeMsg_fromTime(time);
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(R, P, Y);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
}
geometry_msgs::msg::Pose tf_msg2pose(const geometry_msgs::msg::TransformStamped& tf_msg)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = tf_msg.transform.translation.x;
    pose.position.y = tf_msg.transform.translation.y;
    pose.position.z = tf_msg.transform.translation.z;
    pose.orientation.x = tf_msg.transform.rotation.x;
    pose.orientation.y = tf_msg.transform.rotation.y;
    pose.orientation.z = tf_msg.transform.rotation.z;
    pose.orientation.w = tf_msg.transform.rotation.w;
    return pose;
}
tf2::Transform tf_msg2tf(const geometry_msgs::msg::TransformStamped& tf_msg)
{
    return pose2tf( tf_msg2pose(tf_msg) );
}
vector<double> tf_msg2vector(const geometry_msgs::msg::TransformStamped& tf_msg)
{
    return pose2vector( tf_msg2pose(tf_msg) );
}
void tf_msg2positionRPY(const geometry_msgs::msg::TransformStamped& tf_msg, double& x, double& y, double& z, double& R, double& P, double& Y)
{
    vector<double> v = pose2vector( tf_msg2pose(tf_msg) );
    x = v[0];
    y = v[1];
    z = v[2];
    R = v[3];
    P = v[4];
    Y = v[5];
}









geometry_msgs::msg::Pose mean(const vector<geometry_msgs::msg::Pose>& poses)
{
    if(poses.size() == 0)
    {
        printf("Error in mean, the size of input poses is 0, return empty");
        return zero_pose();
    }
    geometry_msgs::msg::Pose pose_mean = zero_pose();
    for(unsigned int i=0; i<poses.size(); i++)
    {
        pose_mean.position.x += poses[i].position.x / (double)poses.size();
        pose_mean.position.y += poses[i].position.y / (double)poses.size();
        pose_mean.position.z += poses[i].position.z / (double)poses.size();
        pose_mean.orientation.w += poses[i].orientation.w / (double)poses.size();
        pose_mean.orientation.x += poses[i].orientation.x / (double)poses.size();
        pose_mean.orientation.y += poses[i].orientation.y / (double)poses.size();
        pose_mean.orientation.z += poses[i].orientation.z / (double)poses.size();
    }
    return pose_mean;
}
tf2::Transform mean(const vector<tf2::Transform>& tfs)
{
    if(tfs.size() == 0)
    {
        printf("Error in mean, the size of input tranforms is 0, return empty");
        return zero_tf();
    }
    geometry_msgs::msg::Pose pose_mean = zero_pose();
    for(unsigned int i=0; i<tfs.size(); i++)
    {
        geometry_msgs::msg::Pose pose = tf2pose(tfs[i]);
        pose_mean.position.x += pose.position.x / (double)tfs.size();
        pose_mean.position.y += pose.position.y / (double)tfs.size();
        pose_mean.position.z += pose.position.z / (double)tfs.size();
        pose_mean.orientation.w += pose.orientation.w / (double)tfs.size();
        pose_mean.orientation.x += pose.orientation.x / (double)tfs.size();
        pose_mean.orientation.y += pose.orientation.y / (double)tfs.size();
        pose_mean.orientation.z += pose.orientation.z / (double)tfs.size();
    }
    return pose2tf(pose_mean);
}
vector<double> mean(const vector< vector<double> >& vectors)
{
    if(vectors.size() == 0)
    {
        printf("Error in mean, the size of input vectors is 0, return empty");
        return zero_vector();
    }
    geometry_msgs::msg::Pose pose_mean = zero_pose();
    for(unsigned int i=0; i<vectors.size(); i++)
    {
        geometry_msgs::msg::Pose pose = vector2pose(vectors[i]);
        pose_mean.position.x += pose.position.x / (double)vectors.size();
        pose_mean.position.y += pose.position.y / (double)vectors.size();
        pose_mean.position.z += pose.position.z / (double)vectors.size();
        pose_mean.orientation.w += pose.orientation.w / (double)vectors.size();
        pose_mean.orientation.x += pose.orientation.x / (double)vectors.size();
        pose_mean.orientation.y += pose.orientation.y / (double)vectors.size();
        pose_mean.orientation.z += pose.orientation.z / (double)vectors.size();
    }
    return pose2vector(pose_mean);
}
void show_vector(const vector<double>& v)
{
    /*printf("\n");
    for(int i=0; i<v.size();i++)
    {
        printf("%.2f,", v[i]);
    }printf("\n");*/
    RCLCPP_INFO(rclcpp::get_logger("tf_utli"), "pose_vec: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", v[0], v[1], v[2], v[3], v[4], v[5]);
}
void show_pose(const geometry_msgs::msg::Pose& pose_in)
{
    show_vector(pose2vector(pose_in));
}
void show_tf(const tf2::Transform& tf_in)
{
    show_vector(tf2vector(tf_in));
}
double pi2pi(double angle)
{
    double new_angle = angle;
    int i = 0;
    while(!(new_angle < PI && new_angle > -PI))
    {
        if(new_angle < -PI)
            new_angle += 2*PI;
        else if(new_angle > PI)
            new_angle -= 2*PI;
        else
            break;
        i++;
        if(i>100)
        {printf("pi2pi error\n"); return new_angle;}
    }
    return new_angle;
}
double pi_2_2pi(double angle)
{
    double new_angle = pi2pi(angle);
    if(new_angle < 0)
        new_angle += 2*PI;
    return new_angle;
}
double angles_diff(double angle1, double angle2)
{
    double angle_dis_1 = pi2pi(angle1 - angle2);
    double angle_dis_2 = pi2pi(angle1 + 2*PI - angle2);
    double angle_dis_3 = pi2pi(angle1 - 2*PI - angle2);

    double angles_diff;
    if(fabs(angle_dis_1) < fabs(angle_dis_2)){
        angles_diff = angle_dis_1;
    }else{
        angles_diff = angle_dis_2;
    }

    if(fabs(angles_diff) > fabs(angle_dis_3)){
        angles_diff = angle_dis_3;
    }

    return angles_diff;
}

builtin_interfaces::msg::Time getTimeMsg()
{
    return rclcpp::Clock{}.now();
}
builtin_interfaces::msg::Time getTimeMsg(const std::shared_ptr<rclcpp::Node>& node)
{
    return node->now();
}
builtin_interfaces::msg::Time getTimeMsg_fromTime(double time)
{
    builtin_interfaces::msg::Time timeMsg;
    timeMsg.sec = (int)time;
    timeMsg.nanosec = (int)((time - timeMsg.sec) * 1e9);
    return timeMsg;
}
builtin_interfaces::msg::Time getTimeMsg_fromTFTime(const tf2::TimePoint& time)
{
    return tf2_ros::toMsg(time);
}
double getTime()
{
    return rclcpp::Clock{}.now().seconds();
}
double getTime(const std::shared_ptr<rclcpp::Node>& node)
{
    return node->now().seconds();
}
double getTime_fromMsg(const builtin_interfaces::msg::Time& time)
{
    return time.sec + time.nanosec * 1e-9;
}
double getTime_fromTFTime(const tf2::TimePoint& time)
{
    return getTime_fromMsg(tf2_ros::toMsg(time));
}
tf2::TimePoint getTFTime_now()
{
    return getTFTime_fromTimeMsg(getTimeMsg());
}
tf2::TimePoint getTFTime_now(const std::shared_ptr<rclcpp::Node>& node)
{
    return getTFTime_fromTimeMsg(node->now());
}
tf2::TimePoint getTFTime_fromTimeMsg(const builtin_interfaces::msg::Time& time)
{
    return tf2_ros::fromMsg(time);
}
tf2::TimePoint getTFTime_fromTime(double time)
{
    return tf2_ros::fromMsg(getTimeMsg_fromTime(time));
}
#endif /* TF_UTIL_H_ */

