#ifndef LIOM_SUBMAP_H
#define LIOM_SUBMAP_H
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#define SUBMAP_INSERT_DISTANT_TH 0.1
#define SUBMAP_INSERT_YAW_TH 0.1
namespace liom{

struct ScanInfo{
    tf2::Transform local_pose;
    bool is_key_scan;
};
class SubMap
{
private:
    tf2::Transform global_pose_; //相对于全局地图的位姿
    pcl::PointCloud<pcl::PointXYZ> map_cloud_;
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_;
    std::vector<ScanInfo> trace_;
    bool insertion_finished_ = false;
    int num_range_data_ = 0; //插入的Scan数量
    int last_key_scan_index_;
    bool IsDeviatioEnough(const tf2::Transform& current_pose,const tf2::Transform& last_pose,const double dis_th,const double yaw_th);
public:
    int num_range_data() const { return num_range_data_; }
    bool insertion_finished() const { return insertion_finished_; }
    void InsertPointcloud(const tf2::Transform& scan2scan_trans,pcl::PointCloud<pcl::PointXYZ>& cloud);
    void Init(const tf2::Transform& global_pose,pcl::PointCloud<pcl::PointXYZ>& cloud);
    void GetMapCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr& map_ptr);
    SubMap(/* args */);
    ~SubMap();
    pcl::PointCloud<pcl::PointXYZ> pub_cloud;
};

}
#endif