#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

#include <pcl/kdtree/kdtree_flann.h>

#include "sample/common/interface.h"

namespace Sample
{
// StatisticalOutlierRemoval
class SOR : public Interface
{
 public:
  SOR() {}
  void setMeanK(int nni) { nn = nni; }
  void setStddevMulThresh(double smt) { mul = smt; }
  int sample();

 private:
  int nn     = 30; // num of neighbors
  double mul = 0.5;
};
} // namespace Sample

#endif