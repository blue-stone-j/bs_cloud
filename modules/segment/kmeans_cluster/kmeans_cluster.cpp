
#include <random>

#include "segment/kmeans_cluster/kmeans_cluster.h"


namespace bcloud
{
namespace segment
{
double distance2(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
  return (a.getVector3fMap( ) - b.getVector3fMap( )).squaredNorm( );
}

KMeansCluster::KMeansCluster( )
{
}

void KMeansCluster::computeKMeansCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud, int k,
                                         int max_iters,
                                         std::vector<int> &assignments,
                                         std::vector<pcl::PointXYZ> &centers)
{
  const int n = cloud.size( );
  assignments.resize(n);

  // Randomly initialize cluster centers
  std::mt19937 gen(42); // fixed seed
  std::uniform_int_distribution<> dis(0, n - 1);
  centers.clear( );
  for (int i = 0; i < k; ++i)
  {
    centers.push_back(cloud[dis(gen)]);
  }

  for (int iter = 0; iter < max_iters; ++iter)
  {
    bool changed = false;

    // Step 1: Assign cloud to closest cluster
    for (int i = 0; i < n; ++i)
    {
      double min_dist  = std::numeric_limits<double>::max( );
      int best_cluster = 0;
      for (int j = 0; j < k; ++j)
      {
        double d = distance2(cloud[i], centers[j]);
        if (d < min_dist)
        {
          min_dist     = d;
          best_cluster = j;
        }
      }
      if (assignments[i] != best_cluster)
      {
        changed        = true;
        assignments[i] = best_cluster;
      }
    }

    // Step 2: Update cluster centers
    std::vector<pcl::PointXYZ> new_centers(k);
    std::vector<int> counts(k, 0);
    for (int i = 0; i < n; ++i)
    {
      new_centers[assignments[i]].x += cloud[i].x;
      new_centers[assignments[i]].y += cloud[i].y;
      new_centers[assignments[i]].z += cloud[i].z;
      counts[assignments[i]] += 1;
    }
    for (int j = 0; j < k; ++j)
    {
      if (counts[j] > 0)
      {
        centers[j].x = new_centers[j].x / counts[j];
        centers[j].y = new_centers[j].y / counts[j];
        centers[j].z = new_centers[j].z / counts[j];
      }
    }

    if (!changed)
    {
      std::cout << "Converged in " << iter + 1 << " iterations\n";
      break;
    }
  }
}

} // namespace segment
} // namespace bcloud