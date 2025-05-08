#include "sample/statistical_outlier_removal/statistical_outlier_removal.h"

namespace bcloud
{
namespace sample
{
int SOR::sample()
{
  pcl::PointCloud<pcl::PointXYZ>().swap(*cloud_filtered);
  std::vector<double> distances(cloud->size());
  double dis_sqr = 0;
  pcl::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> searcher(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  searcher->setInputCloud(cloud);

  int valid_distances = 0;
  double dis_mean     = 0;
  std::vector<int> pt_ind;                   // save index of element from kd-tree
  std::vector<float> pt_dis;                 // save distance
  for (size_t i = 0; i < cloud->size(); ++i) // iii = input indices iterator
  {
    // Perform the nearest k search
    if (searcher->nearestKSearch(cloud->points[i], nn + 1, pt_ind, pt_dis) == 0)
    {
      distances[i] = 1000; // set a large value to remove this point
      continue;
    }

    // Calculate the mean distance to its neighbors
    double dist_sum = 0.0;
    for (int k = 1; k < nn + 1; ++k) // k = 0 is the query point
    {
      dist_sum += sqrt(pt_dis[k]);
    }
    distances[i] = dist_sum / nn;
    dis_sqr += distances[i] * distances[i];
    dis_mean += distances[i];
    valid_distances++;
  }
  double variance = (dis_sqr - dis_mean * dis_mean / valid_distances) / (valid_distances - 1);
  double stddev   = sqrt(variance);
  dis_mean /= valid_distances; // standard deviation
  double threshold = dis_mean + mul * stddev;

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (distances[i] < threshold)
    {
      cloud_filtered->push_back(cloud->points[i]);
    }
  }

  return 0;
}
} // namespace sample
} // namespace bcloud
