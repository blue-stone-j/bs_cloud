#include "sample_filter/farthest_point_sample.h"

namespace SampleFilter
{
FarthestPointFilter::FarthestPointFilter(FarthestPointFilterParams params) :
  params(params) {}

int FarthestPointFilter::sample_filter()
{
  std::vector<int> idxs(params.number_batch * params.number_select);
  std::vector<float> temp(params.number_batch * params.size_batch);
  for (int i = 0; i < params.number_batch; ++i)
  {
    int index                      = 0;
    idxs[i * params.number_select] = index;
    for (int j = 1; j < params.number_select; ++j) // Select m points
    {
      int best_i = 0;
      float best = -1.0f;

      const pcl::PointXYZ &p1 = cloud->points[i * params.size_batch + index];

      for (int k = 0; k < params.size_batch; ++k)
      {
        const pcl::PointXYZ &p2 = cloud->points[i * params.size_batch + k];
        float d                 = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z);

        float d2                        = std::min(d, temp[i * params.size_batch + k]);
        temp[i * params.size_batch + k] = d2;

        if (d2 > best)
        {
          best   = d2;
          best_i = k;
        }
      }

      index                              = best_i;
      idxs[i * params.number_select + j] = index;
    }
  }

  for (const auto &idx : idxs)
  {
    cloud_filtered->points.emplace_back(cloud->points[idx]);
  }
  return 0;
}

float GetDistance(pcl::PointXYZ &p1, pcl::PointXYZ &p2)
{
  return std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

pcl::PointCloud<pcl::PointXYZ> GetFPS(pcl::PointCloud<pcl::PointXYZ> &input, const int num)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  output.emplace_back(input[0]);
  auto tmp = input;
  tmp.erase(tmp.begin());
  while (static_cast<int>(output.size()) <= num)
  {
    float max_distance = 0.f;
    int index          = 0;
    for (auto it = tmp.begin(); it != tmp.end(); ++it)
    {
      float min_distance = 0.f;
      for (auto it2 = output.begin(); it2 != output.end(); ++it2)
      {
        min_distance = std::min(GetDistance(*it, *it2), min_distance);
      }
      if (max_distance < min_distance)
      {
        max_distance = min_distance;
        index        = it - tmp.begin();
      }
    }
    output.emplace_back(tmp[index]);
    tmp.erase(tmp.begin() + index);
  }
  return output;
}
} // namespace SampleFilter