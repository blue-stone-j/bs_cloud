#include "sample/far_outliers_removal/far_outliers_removal.h"

namespace bcloud
{
namespace sample
{
FarOutliersRemoval::FarOutliersRemoval()
{}


FarOutliersRemoval::FarOutliersRemoval(FarOutliersRemovalParams params)
{
  params_ = params;
}
int FarOutliersRemoval::sample() { return 0; }
} // namespace sample
} // namespace bcloud
