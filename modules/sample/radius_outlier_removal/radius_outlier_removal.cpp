
#include <pcl/kdtree/kdtree_flann.h>

#include "sample/radius_outlier_removal/radius_outlier_removal.h"

namespace bcloud
{
namespace sample
{
int ROR::sample()
{
  std::vector<int> pt_ind;   // save index of element from kd-tree
  std::vector<float> pt_dis; // save distance
  pcl::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> searcher(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  searcher->setInputCloud(cloud);
  std::cout << r << ", " << num_threshold << std::endl;
  for (size_t i = 0; i < cloud->size(); ++i) // iii = input indices iterator
  {
    // Perform the nearest search
    if (searcher->radiusSearch(cloud->points[i], r, pt_ind, pt_dis) > num_threshold)
    {
      cloud_filtered->push_back(cloud->points[i]);
    }
  }

  return 0;
}
void ROR::setRadiusSearch(double ri) { r = ri; }
void ROR::setMinNeighborsInRadius(int num) { num_threshold = num; }
} // namespace sample
} // namespace bcloud
