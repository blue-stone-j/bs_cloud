#ifndef FAR_OUTLIERS_REMOVAL_H
#define FAR_OUTLIERS_REMOVAL_H

#include "sample/common/interface.h"

namespace bcloud
{
namespace sample
{
struct FarOutliersRemovalParams
{
  float ratio  = 0.2;
  float minDis = 5, maxDis = 100, thrDis = 3;
};

class FarOutliersRemoval : public Interface
{
 public:
  FarOutliersRemoval();
  FarOutliersRemoval(FarOutliersRemovalParams params);
  int sample();

 private:
  FarOutliersRemovalParams params_;
};
} // namespace sample
} // namespace bcloud

#endif