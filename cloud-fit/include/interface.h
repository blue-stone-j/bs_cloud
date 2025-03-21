#ifndef INTERFACE_H
#define INTERFACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Interface
{
 public:
  pcl::PointCloud<pcl::PointXYZ> cloud;
  virtual int fit() = 0;
  std::vector<float> model_coefficients;
};


#endif