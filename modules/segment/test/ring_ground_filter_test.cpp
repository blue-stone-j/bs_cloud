
#include <Eigen/Core>
#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include "segment/ring_ground_filter/ring_ground_filter.h"

TEST(ring_ground_filter_test, test1)
{
  // 加载点云
  pcl::PointCloud<bcloud::segment::PointXYZIR>::Ptr cloud(new pcl::PointCloud<bcloud::segment::PointXYZIR>);
  if (pcl::io::loadPCDFile<bcloud::segment::PointXYZIR>("../assets/cloud/slope.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    ASSERT_TRUE(false);
  }

  bcloud::segment::GroundFilter node;
  pcl::PointCloud<bcloud::segment::PointXYZIR> ground_points;
  node.estimateGround(cloud, ground_points);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS( );
}