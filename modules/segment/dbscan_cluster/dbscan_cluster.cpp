
#include "segment/dbscan_cluster/dbscan_cluster.h"

namespace bcloud
{
namespace segment
{
DBSCANCluster::DBSCANCluster( )
{
}

void DBSCANCluster::computeDBSCANCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<std::vector<int>> &clusters,
                                         double distance_threshold, int noise_threshold)
{
  if (cloud.empty( ))
  {
    return; // Not enough points to form clusters
  }

  enum class Label
  {
    VISITED   = 1,
    UNVISITED = 0,
    NOISE     = -1
  };

  const int n = cloud.size( );
  std::vector<Label> visited(n, Label::UNVISITED);
  std::vector<int> cluster_labels(n, -1); // -1 means noise
  int cluster_id = 0;

  for (int i = 0; i < n; ++i)
  {
    if (visited[i] != Label::UNVISITED)
    {
      continue;
    }

    visited[i] = Label::NOISE;

    // Find neighbors
    constexpr double kClusterDistance = 1.0;
    std::vector<int> neighbors;
    for (int j = 0; j < n; ++j)
    {
      if (visited[j] != Label::UNVISITED)
      {
        continue;
      }
      if (std::sqrt((cloud[i].x - cloud[j].x) * (cloud[i].x - cloud[j].x)
                    + (cloud[i].y - cloud[j].y) * (cloud[i].y - cloud[j].y)
                    + (cloud[i].z - cloud[j].z) * (cloud[i].z - cloud[j].z))
          <= distance_threshold)
      {
        neighbors.emplace_back(j);
      }
    }

    constexpr int kNoiseThreshold = 5; // Minimum points to form a cluster
    if (neighbors.size( ) < noise_threshold)
    {
      continue; // Noise
    }

    for (auto &neighbor : neighbors)
    {
      visited[neighbor] = Label::NOISE;
    }


    // Start a new cluster
    std::vector<int> cluster;
    cluster.emplace_back(i);
    cluster_labels[i] = cluster_id;

    // Expand cluster
    std::vector<int> seed_queue = neighbors;
    for (size_t q = 0; q < seed_queue.size( ); ++q)
    {
      int idx = seed_queue[q];

      // Find neighbors of this point
      std::vector<int> neighbor_points;
      for (int j = 0; j < n; ++j)
      {
        if (visited[j] != Label::UNVISITED)
        {
          continue;
        }
        if (std::sqrt((cloud[idx].x - cloud[j].x) * (cloud[idx].x - cloud[j].x)
                      + (cloud[idx].y - cloud[j].y) * (cloud[idx].y - cloud[j].y)
                      + (cloud[idx].z - cloud[j].z) * (cloud[idx].z - cloud[j].z))
            <= kClusterDistance)
        {
          visited[j] = Label::NOISE;
          neighbor_points.emplace_back(j);
        }
      }

      if (neighbor_points.size( ) >= static_cast<size_t>(kNoiseThreshold))
      {
        seed_queue.insert(seed_queue.end( ), neighbor_points.begin( ), neighbor_points.end( ));
      }


      if (cluster_labels[idx] == -1)
      {
        cluster.emplace_back(idx);
        cluster_labels[idx] = cluster_id;
      }
    } // endfor: cluster expansion

    clusters.emplace_back(cluster);

    ++cluster_id;
  }
}

} // namespace segment
} // namespace bcloud
