#ifndef LIOM_CORE_H_
#define LIOM_CORE_H_
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "submap.h"
#include "tfcalculate.h"
#include "scanmatch.h"
namespace liom{

class Core{
public:
    Core();
    ~Core();
    void HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan);
private:
    laser_geometry::LaserProjection projector_;
    TfCalculate tfcalculate;
    SubMap submap;
    ScanMatch scanmatch;
};
}

#endif //LIOM_CORE_H_