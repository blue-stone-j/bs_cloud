#ifndef RADIUS_OUTLIER_REMOVAL_H
#define RADIUS_OUTLIER_REMOVAL_H

#include "sample/common/interface.h"

namespace bcloud
{
namespace sample
{
class ROR : public Interface
{
 public:
  int sample() override;
  void setRadiusSearch(double ri);
  void setMinNeighborsInRadius(int num);

 private:
  double r          = 0.3;
  int num_threshold = 5;
};

} // namespace sample
} // namespace bcloud



#endif