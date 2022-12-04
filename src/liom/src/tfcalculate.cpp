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
    tf2::Matrix3x3 basis;
    tf2::Vector3 origin;
     try{

            current_pose_stamped = tfBuffer_.lookupTransform("odom", "rslidar",ros::Time(0));  //不能是baselink;
            tf2::fromMsg(current_pose_stamped.transform,current_pose); //更新上一次位姿
            
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("get tf error: %s",ex.what());
            return;
        }
    if(!initialized_){
        initialized_ = true;
        basis.setRPY(0,0,0);
        relative_pose.setBasis(basis);
        origin.setX(0);
        origin.setY(0);
        origin.setZ(0);
        relative_pose.setOrigin(origin);
        first_guess_pose = relative_pose;
    }
    else{
        relative_pose = last_pose_.inverse() * current_pose;
        first_guess_pose = relative_pose;

    }
    // std::cout<<"tf: current_pose xyz" << current_pose.getOrigin().getX()<<" "<<current_pose.getOrigin().getY()<<" "<<current_pose.getOrigin().getZ()<<""<<std::endl;
    // std::cout<<"tf: relative_pose xyz" << relative_pose.getOrigin().getX()<<" "<<relative_pose.getOrigin().getY()<<" "<<relative_pose.getOrigin().getZ()<<""<<std::endl;
    // std::cout<<"tf: last pose xyz" << last_pose_.getOrigin().getX()<<" "<<last_pose_.getOrigin().getY()<<" "<<last_pose_.getOrigin().getZ()<<""<<std::endl;
   
    // double roll,pitch,yaw;
    // current_pose.getBasis().getRPY(roll,pitch,yaw);
    // std::cout<<"tf: current_pose rpy " <<roll<<" "<<pitch<<" "<<yaw<<std::endl;
    // relative_pose.getBasis().getRPY(roll,pitch,yaw);
    // std::cout<<"tf: relative_pose rpy " <<roll<<" "<<pitch<<" "<<yaw<<std::endl;
    // last_pose_.getBasis().getRPY(roll,pitch,yaw);
    // std::cout<<"tf: last_pose_ rpy " <<roll<<" "<<pitch<<" "<<yaw<<std::endl;


    // //测试代码
    // static tf2::Transform odom_pose;
    // static bool is_odom_init = false;
    // if(is_odom_init == false)
    // {
    //     is_odom_init = true;
    //     basis.setRPY(0,0,0);
    //     odom_pose.setBasis(basis);
    //     origin.setX(0);
    //     origin.setY(0);
    //     origin.setZ(0);
    //     odom_pose.setOrigin(origin);

    // }
    // else
    // {
    //     odom_pose = odom_pose * relative_pose;
    //     std::cout<<"tf: odom_pose_reback" << odom_pose.getOrigin().getX()<<" "<<odom_pose.getOrigin().getY()<<""<<odom_pose.getOrigin().getZ()<<std::endl;
        
    // }
    ////测试代码结束
    last_pose_ = current_pose;
 }

}