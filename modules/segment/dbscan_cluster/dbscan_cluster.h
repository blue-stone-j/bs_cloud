
#ifndef DBSCAN_CLUSTER_H
#define DBSCAN_CLUSTER_H

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace bcloud
{
namespace segment
{
class DBSCANCluster
{
 public:
  DBSCANCluster( );
  // get index of points and size of this cloud
  void computeDBSCANCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<std::vector<int>> &clusters,
                            double distance_threshold, int noise_threshold);
};

} // namespace segment
} // namespace bcloud

#endif