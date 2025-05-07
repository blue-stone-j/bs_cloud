#include "sample/far_outliers_removal/far_outliers_removal.h"

namespace Sample
{
FarOutliersRemoval::FarOutliersRemoval()
{}


FarOutliersRemoval::FarOutliersRemoval(FarOutliersRemovalParams params)
{
  params_ = params;
}
int FarOutliersRemoval::sample() { return 0; }
} // namespace Sample