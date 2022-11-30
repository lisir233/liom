#ifndef LIOM_NODE_H_
#define LIOM_NODE_H_
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>

namespace liom{

class Node{
public:
    Node();
    ~Node();
private:
    
    ::ros::NodeHandle node_handle_;

    ::ros::Publisher submap_publisher_;  //发布子图

    ::ros::Subscriber scan_subscriber_;  //订阅scan

    ::tf::TransformListener tf_listener;  

    ::tf::TransformBroadcaster tf_broadcaster;

    laser_geometry::LaserProjection projector_;
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};
}
#endif //LIOM_NODE_H_