#include "submap.h"
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include<cmath>
#include<iostream>
namespace liom{
SubMap::SubMap(/* args */)
{
    
}

SubMap::~SubMap()
{
}
bool SubMap::IsDeviatioEnough(const tf2::Transform& current_pose,const tf2::Transform& last_pose,const double dis_th,const double yaw_th)
{
    tf2::Transform  diff = last_pose.inverse()*current_pose;
    double roll,pitch,yaw;
    diff.getBasis().getRPY(roll,pitch,yaw);
    double x,y,len;
    x = diff.getOrigin().getX();
    y = diff.getOrigin().getY();
    len = sqrt(x*x+y*y);
    // std::cout<<"current_pose xyz"<<current_pose.getOrigin().getX()<<" "<<current_pose.getOrigin().getY()<<" "<<current_pose.getOrigin().getZ()<<" ";
    // std::cout<<"last_pose xyz"<<last_pose.getOrigin().getX()<<" "<<last_pose.getOrigin().getY()<<" "<<last_pose.getOrigin().getZ()<<" "<<std::endl;
    // std::cout<<"diff xyz"<<diff.getOrigin().getX()<<" "<<diff.getOrigin().getY()<<" "<<diff.getOrigin().getZ()<<" "<<std::endl;
    // std::cout<<"len"<<len<<"yaw"<<yaw<<std::endl;
    return (len> dis_th || abs(yaw) > yaw_th);

}
void SubMap::InsertPointcloud(const tf2::Transform& scan2scan_trans,pcl::PointCloud<pcl::PointXYZ>& cloud){
    ScanInfo scaninfo;
    ScanInfo last_scaninfo = trace_.back();
    tf2::Transform local_pose = last_scaninfo.local_pose * scan2scan_trans;
    scaninfo.local_pose = local_pose;

    // std::cout<<"submap last_scaninfo xyz"<<last_scaninfo.local_pose.getOrigin().getX()<<" "<<last_scaninfo.local_pose.getOrigin().getY()<<" "<<last_scaninfo.local_pose.getOrigin().getZ()<<" "<<std::endl;
    // std::cout<<"submap scan2scan_trans xyz"<<scan2scan_trans.getOrigin().getX()<<" "<<scan2scan_trans.getOrigin().getY()<<" "<<scan2scan_trans.getOrigin().getZ()<<" "<<std::endl;
    // std::cout<<"submap local_pose xyz"<<local_pose.getOrigin().getX()<<" "<<local_pose.getOrigin().getY()<<" "<<local_pose.getOrigin().getZ()<<" "<<std::endl;
    // std::cout<<"submap last_key_scan_index_"<<last_key_scan_index_<<std::endl;  

    // double roll,pitch,yaw;
    // last_scaninfo.local_pose.getBasis().getRPY(roll,pitch,yaw);
    // std::cout<<"submap last_scaninfo rpy"<<roll<<" "<<pitch<<" "<<yaw<<" "<<std::endl;
    // scan2scan_trans.getBasis().getRPY(roll,pitch,yaw);
    // std::cout<<"submap scan2scan_trans rpy"<<roll<<" "<<pitch<<" "<<yaw<<" "<<std::endl;
    // local_pose.getBasis().getRPY(roll,pitch,yaw);
    // std::cout<<"submap local_pose rpy"<<roll<<" "<<pitch<<" "<<yaw<<" "<<std::endl;



    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    if(IsDeviatioEnough(local_pose,trace_[last_key_scan_index_].local_pose,SUBMAP_INSERT_DISTANT_TH,SUBMAP_INSERT_YAW_TH))
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_trans;
        trans.translation()<<local_pose.getOrigin().getX(), local_pose.getOrigin().getY(), local_pose.getOrigin().getZ();
        double roll, pitch, yaw;//定义存储r\p\y的容器
        local_pose.getBasis().getRPY(roll,pitch,yaw);
        trans.rotate(Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (cloud, cloud_trans, trans);
        map_cloud_ = map_cloud_ + cloud_trans;
        std::cout<< "SubMap :hight: " <<map_cloud_.height<< "width: " <<map_cloud_.width<<"size:"<<map_cloud_.points.size()<<std::endl;

        last_key_scan_index_ = trace_.size() - 1;
        scaninfo.is_key_scan = true;
        trace_.push_back(scaninfo);
    }
    else
    {
        scaninfo.is_key_scan = false;
        trace_.push_back(scaninfo);
    }
}
void SubMap::Init(const tf2::Transform& global_pose,pcl::PointCloud<pcl::PointXYZ>& cloud){
    
    ScanInfo scaninfo;
    global_pose_ = global_pose;
    //第一帧地图永远是0，0，0，0，0，0
    tf2::Transform  pose;
    tf2::Matrix3x3 basis;
    tf2::Vector3 origin;
    basis.setRPY(0,0,0);
    pose.setBasis(basis);
    origin.setX(0);
    origin.setY(0);
    origin.setZ(0);
    pose.setOrigin(origin);

    scaninfo.local_pose = pose;
    scaninfo.is_key_scan = true;
    trace_.push_back(scaninfo);
    map_cloud_ = cloud;
    std::cout<< "SubMap :hight: " <<map_cloud_.height<< "width: " <<map_cloud_.width<<"size:"<<map_cloud_.points.size()<<std::endl;
    last_key_scan_index_ = 0;
    num_range_data_++;
    
}
void SubMap::GetMapCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr& map_ptr){
    map_ptr = map_cloud_.makeShared();

}


}