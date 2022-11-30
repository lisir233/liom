#include <liom_core.h>
#include <laser_geometry/laser_geometry.h>
#include "submap.h"
namespace liom{
Core::Core(){

}
void Core::HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //1.转换成点云
    sensor_msgs::PointCloud2 cloud;


    tf2::Transform global_pose;
    tf2::Transform scan2scan_pose;

    projector_.projectLaser(*scan, cloud);     
    //2.点云匹配
    
    //如果上一个子地图已经完成
    if(submap.num_range_data() == 0){
        submap.Init(global_pose,cloud);
    }
    else{
        submap.InsertPointcloud(scan2scan_pose,cloud);
    }




    //2.2获取当前帧姿态
    //2.3将信息传入
    //3.将点云插入子图点云中
}
 
} //namespace liom_core