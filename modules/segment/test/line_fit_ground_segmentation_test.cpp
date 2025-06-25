#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include "segment/line_fit_ground_segmentation/line_fit_ground_segmentation.h"

TEST(line_fit_ground_segmentation_test, test1)
{
  std::string cloud_file = "../assets/cloud/slope.pcd";
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_file, cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file \n");
    ASSERT_TRUE(false);
  }
  pcl::io::loadPCDFile(cloud_file, cloud);
  bcloud::segment::GroundSegmentationParams params;
  bcloud::segment::GroundSegmentation segmenter(params);
  std::vector<int> labels;

  segmenter.estimateGround(cloud, &labels);
}
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS( );
}