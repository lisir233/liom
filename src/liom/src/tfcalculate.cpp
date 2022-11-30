#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs.h>
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
    geometry_msgs::TransformStamped transformStamped;
    if(!initialized_){
        try{

            transformStamped = tfBuffer_.lookupTransform("/odom", "/rslidar",ros::Time(0));
        
        }
        catch(tf2::TransformException &ex){
        //如果妹扒拉出来消息，显示警告消息
            ROS_WARN("tf2::TransformException %s",ex.what());
            return;
        }
         tf2::fromMsg(transformStamped,last_pose_);
        initialized_ = true;
    }
    else
    {
        ;
    }
 }
}