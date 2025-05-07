#ifndef FARTHEST_POINT_SAMPLE_H
#define FARTHEST_POINT_SAMPLE_H

#include <cmath>

#include "sample/common/interface.h"

namespace Sample
{
/*
todo: divide cloud into batchhes
total: n in one batch
set A: m
set B: n-m
b: batch
dataset: (b, n, 3)
temp: (b, n)
idxs: (b, m)
*/

float GetDistance(pcl::PointXYZ &p1, pcl::PointXYZ &p2);

pcl::PointCloud<pcl::PointXYZ> GetFPS(pcl::PointCloud<pcl::PointXYZ> &input, const int num);

struct FarthestPointFilterParams
{
  int number_batch  = 1;
  int number_select = 1024;
  int size_batch    = 1024;
};

class FarthestPointFilter : public Interface
{
 public:
  FarthestPointFilterParams params;
  FarthestPointFilter() {}
  FarthestPointFilter(FarthestPointFilterParams params);
  int sample();
};

} // namespace Sample

#endif