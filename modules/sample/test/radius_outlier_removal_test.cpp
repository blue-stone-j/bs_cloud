#include <gtest/gtest.h>

#include <pcl/filters/radius_outlier_removal.h>

#include "sample/radius_outlier_removal/radius_outlier_removal.h"

TEST(radius_outlier_removal_test, Cloud)
{
  bcloud::sample::ROR ror;
  ror.setCloudPath("../assets/cloud/fog.pcd");

  // pcl
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(ror.cloud);
  outrem.setRadiusSearch(0.3);       // 设置搜索半径
  outrem.setMinNeighborsInRadius(5); // 设置半径内最小邻近点数
  outrem.filter(cloud_filtered);
  pcl::io::savePCDFileBinary("../result/filtered_point_cloud0.pcd", cloud_filtered);
  std::cout << "pcl: " << ror.cloud->size() << ", " << cloud_filtered.size() << std::endl;

  // customized
  ror.sample();
  ror.saveCloud("../result/filtered_point_cloud1.pcd");
  std::cout << "nne: " << ror.cloud->size() << ", " << ror.cloud_filtered->size() << std::endl;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);



  return RUN_ALL_TESTS();
}