#include "liom_node.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char** argv)
{
    ::ros::init(argc, argv, "liom_node");
    ROS_INFO("liom_node main");
    liom::Node node;
    ::ros::spin();

    return 0;
}
