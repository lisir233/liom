#include "liom_node.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char** argv)
{
    ::ros::init(argc, argv, "liom_node");
    
    ::ros::spin();

    liom::Node node();
    return 0;
}