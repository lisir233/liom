#include "liom_node.h"
namespace liom{

Node::Node(){
     //订阅　"/scan"
    scan_subscriber_ = node_handle_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &Node::scanCallback, this);
    submap_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
      
    
}
void Node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    


}

} //namespace liom
