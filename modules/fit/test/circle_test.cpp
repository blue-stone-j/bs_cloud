#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "fit/circle/circle.h"

TEST(circle_test, test1)
{
  std::string path = "../assets/cloud/plane1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file.pcd \n");
    ASSERT_TRUE(false);
  }

  // customized
  bcloud::fit::CircleFit circle_fit;
  circle_fit.cloud = *cloud;

  circle_fit.fit();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}