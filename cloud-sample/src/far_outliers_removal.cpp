#include "sample_filter/far_outliers_removal.h"

namespace SampleFilter
{
FarOutliersRemoval::FarOutliersRemoval()
{}


FarOutliersRemoval::FarOutliersRemoval(FarOutliersRemovalParams params)
{
  params_ = params;
}
int FarOutliersRemoval::sample_filter() { return 0; }
} // namespace SampleFilter