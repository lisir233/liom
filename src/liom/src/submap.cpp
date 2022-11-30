#include "submap.h"
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
namespace liom{
SubMap::SubMap(/* args */)
{

}

SubMap::~SubMap()
{
}

void SubMap::InsertPointcloud(const tf2::Transform& scan2scan_trans,sensor_msgs::PointCloud2& cloud){
  
}
void SubMap::Init(const tf2::Transform& global_pose,sensor_msgs::PointCloud2& cloud){
    ScanInfo scaninfo;
    global_pose_ = global_pose;

    tf2::Transform  pose;
    scaninfo.local_pose = pose;
    scaninfo.is_key_scan = true;
    trace_.push_back(scaninfo);

    map_cloud_ = cloud;
    num_range_data_++;
}

}