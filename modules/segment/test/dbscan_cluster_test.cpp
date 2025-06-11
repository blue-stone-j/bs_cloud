#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include "segment/dbscan_cluster/dbscan_cluster.h"


TEST(Test, test1)
{
  std::string path = "../assets/cloud/plane1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file.pcd \n");
    ASSERT_TRUE(false);
  }

  bcloud::segment::DBSCANCluster dbscan;
  std::vector<std::vector<int>> clusters;
  dbscan.computeDBSCANCluster(*cloud, clusters, 0.5, 5);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS( );
}