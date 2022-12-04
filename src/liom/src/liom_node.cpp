#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "liom_node.h"
#include "liom_core.h"
namespace liom{

Node::Node(){
    ROS_INFO("Node::Node()");
     //订阅　"/scan"
    scan_subscriber_ = node_handle_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &Node::scanCallback, this);
    submap_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
      
    
}
Node::~Node(){}
void Node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

        //std::cout<<"scancallback"<<std::endl;
        core.HandleLaserScanMessage(scan);
        
        pcl::PointCloud<pcl::PointXYZ>* cloud_ptr;
         pub_cloud_.data.clear();
         pcl::toROSMsg(core.pubcloud, pub_cloud_);
         pub_cloud_.header.frame_id = "odom";
         submap_publisher_.publish(pub_cloud_);
}

} //namespace liom
