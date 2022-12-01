#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tfcalculate.h"
namespace liom{

TfCalculate::TfCalculate(){

tfListener_ptr = new tf2_ros::TransformListener(tfBuffer_);
}
TfCalculate::~TfCalculate(){
 delete(tfListener_ptr);
}
void TfCalculate::SetFarmeID(std::string& target_frame,std::string& source_frame){
    target_frame_ = target_frame;
    source_frame_ = source_frame;
}
void TfCalculate::GetRelativeTF2LastTime(tf2::Transform &first_guess_pose){
    tf2::Transform relative_pose;
    geometry_msgs::TransformStamped current_pose_stamped;
    tf2::Transform current_pose;
     try{

            current_pose_stamped = tfBuffer_.lookupTransform("/odom", "/rslidar",ros::Time(0));
            tf2::fromMsg(current_pose_stamped.transform,last_pose_); //更新上一次位姿
        
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("get tf error: %s",ex.what());
            return;
        }
    if(!initialized_){
        last_pose_ = current_pose;
        initialized_ = true;
        return;
    }
    else{
        relative_pose = last_pose_.inverse() * current_pose;
        first_guess_pose = relative_pose;
    }
 }
}