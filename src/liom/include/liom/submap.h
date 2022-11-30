#ifndef LIOM_SUBMAP_H
#define LIOM_SUBMAP_H
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/PointCloud2.h>

namespace liom{

struct ScanInfo{
    tf2::Transform local_pose;
    bool is_key_scan;
};
class SubMap
{
private:
    tf2::Transform global_pose_; //相对于全局地图的位姿
    sensor_msgs::PointCloud2 map_cloud_;
    sensor_msgs::PointCloud2 obstacle_cloud_;
    std::vector<ScanInfo> trace_;
    int num_range_data_ = 0; //插入的Scan数量
    bool insertion_finished_ = false;
    
public:
    int num_range_data() const { return num_range_data_; }
    bool insertion_finished() const { return insertion_finished_; }
    void InsertPointcloud(const tf2::Transform& scan2scan_trans,sensor_msgs::PointCloud2& cloud);
    void Init(const tf2::Transform& global_pose,sensor_msgs::PointCloud2& cloud);
    SubMap(/* args */);
    ~SubMap();
};

}
#endif