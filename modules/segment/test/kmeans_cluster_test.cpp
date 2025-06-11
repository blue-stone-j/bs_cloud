#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include "segment/kmeans_cluster/kmeans_cluster.h"

TEST(Test, test1)
{
  std::string path = "../assets/cloud/plane1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file.pcd \n");
    ASSERT_TRUE(false);
  }

  bcloud::segment::KMeansCluster kmeans;
  std::vector<int> assignments;
  std::vector<pcl::PointXYZ> centers;
  kmeans.computeKMeansCluster(*cloud, 3, 100, assignments, centers);
  ASSERT_EQ(centers.size( ), 3);
  ASSERT_EQ(assignments.size( ), cloud->size( ));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS( );
}