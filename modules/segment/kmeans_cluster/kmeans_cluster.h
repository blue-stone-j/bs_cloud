#ifndef KEAMENS_CLUSTER_H
#define KEAMENS_CLUSTER_H

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace bcloud
{
namespace segment
{
class KMeansCluster
{
 public:
  KMeansCluster( );
  // get index of points and size of this cloud
  void computeKMeansCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud, int k,
                            int max_iters,
                            std::vector<int> &assignments,
                            std::vector<pcl::PointXYZ> &centers);
};

} // namespace segment
} // namespace bcloud

#endif