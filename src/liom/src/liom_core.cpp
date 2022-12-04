#include <liom_core.h>
#include <laser_geometry/laser_geometry.h>
#include "submap.h"
#include <pcl_conversions/pcl_conversions.h>
namespace liom{
Core::Core(){

}
Core::~Core(){

}

void Core::HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::PointCloud2 cloudmsg;

    tf2::Transform first_guss_pose;
    tf2::Transform global_pose;
    tf2::Transform scan2scan_pose;
    static int callback_count = 0;
    if(callback_count++ % 10 == 0)
    {
        tfcalculate.GetRelativeTF2LastTime(first_guss_pose);
        scanmatch.Calculate(first_guss_pose,scan,scan2scan_pose);
        //std::cout<<"first guss pose x:"<<first_guss_pose.getOrigin().getX()<<"y: "<<first_guss_pose.getOrigin().getY()<<std::endl;
        //std::cout<<"plicp pose x:"<<scan2scan_pose.getOrigin().getX()<<"y: "<<scan2scan_pose.getOrigin().getY()<<"y: "<<scan2scan_pose.getOrigin().getZ()<<std::endl;
        projector_.projectLaser(*scan, cloudmsg);    
        pcl::PointCloud<pcl::PointXYZ> cloud; 
        pcl::fromROSMsg(cloudmsg,cloud);
        //如果上一个子地图已经完成
        if(submap.num_range_data() == 0){
            submap.Init(global_pose,cloud);  
        }
        else{
            //3.将新点云插入子图点云中
            submap.InsertPointcloud(scan2scan_pose,cloud);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        submap.GetMapCloudPtr(pub_cloud);
        pubcloud = *pub_cloud;
    }
   

   
}

 
} //namespace liom_core