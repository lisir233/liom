#ifndef LIOM_SCANMATCH_H
#define LICM_SCANMATCH_H

#include <cmath>
#include <vector>
#include <chrono>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

// csm
#include <csm/csm_all.h>
#undef min
#undef max
namespace liom{
class ScanMatch
{
private:

    // csm
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;
    ros::Time last_icp_time_;               // 上次的时间戳
    std::vector<double> a_cos_;             // 保存下来雷达各个角度的cos值
    std::vector<double> a_sin_;             // 保存下来雷达各个角度的sin值
    
    bool initialized_;
    void InitParams();
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);
    void ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time);
public:
    ScanMatch(/* args */);
    ~ScanMatch();
    void Calculate(const tf2::Transform &first_guess_pose,const sensor_msgs::LaserScan::ConstPtr &scan_msg,tf2::Transform&  result_pose);
};


}
#endif