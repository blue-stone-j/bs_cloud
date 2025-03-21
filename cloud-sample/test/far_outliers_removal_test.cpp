#include <gtest/gtest.h>

#include "sample_filter/far_outliers_removal.h"

TEST(far_outliers_removal_test, Cloud)
{
  SampleFilter::FarOutliersRemovalParams params;
  SampleFilter::FarOutliersRemoval far_outliers_removal(params);

  far_outliers_removal.setCloudPath("../assets/cloud/fog.pcd");
  far_outliers_removal.sample_filter();
  far_outliers_removal.saveCloud("../result/filtered_far_outliers_removal.pcd");
  std::cout << "nne: " << far_outliers_removal.cloud->size() << ", " << far_outliers_removal.cloud_filtered->size() << std::endl;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}