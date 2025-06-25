
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include "segment/ray_ground_filter/ray_ground_filter.h"

TEST(ray_ground_filter_test, test1)
{
  // 加载点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>("../assets/cloud/slope.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    ASSERT_TRUE(false);
  }

  bcloud::segment::RayGroundFilter app;

  app.estimateGround(cloud);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS( );
}