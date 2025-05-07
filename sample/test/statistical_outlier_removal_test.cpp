#include <gtest/gtest.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "sample/statistical_outlier_removal/statistical_outlier_removal.h"


TEST(statistical_outlier_removal_test, Cloud1)
{
  Sample::SOR sor;
  sor.setCloudPath("../assets/cloud/fog.pcd");

  // pcl
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> my_sor;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  my_sor.setInputCloud(sor.cloud);
  my_sor.setMeanK(30);            // 设置在进行均值和标准差计算时考虑的邻近点个数
  my_sor.setStddevMulThresh(0.5); // 设置标准差倍数阈值: 距离超过 整体平均距离 + 1 × 标准差 的点都会被视为离群点并被移除
  my_sor.filter(cloud_filtered);
  pcl::io::savePCDFileBinary("../result/filtered_point_cloud0.pcd", cloud_filtered);
  std::cout << "pcl: " << sor.cloud->size() << ", " << cloud_filtered.size() << std::endl;

  // customized
  sor.sample();
  sor.saveCloud("../result/filtered_point_cloud1.pcd");
  std::cout << "nne: " << sor.cloud->size() << ", " << sor.cloud_filtered->size() << std::endl;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}