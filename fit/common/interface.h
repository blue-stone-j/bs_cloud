#ifndef FITINTERFACE_H
#define FITINTERFACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace bcloud
{
class FitInterface
{
 public:
  pcl::PointCloud<pcl::PointXYZ> cloud;
  virtual int fit() = 0;
  std::vector<float> model_coefficients;
};
} // namespace bcloud

#endif