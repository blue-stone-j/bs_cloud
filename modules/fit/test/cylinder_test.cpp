#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include "fit/cylinder/cylinder.h"

TEST(cylinder_test, test1)
{
  std::string path = "../assets/cloud/cylinder1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file.pcd \n");
    ASSERT_TRUE(false);
  }

  // pcl
  //--------------------------------normal-----------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setSearchMethod(tree);
  normal_est.setInputCloud(cloud);
  normal_est.setKSearch(50);
  normal_est.compute(*cloud_normals);
  //------------------------------create segment---------------------------------
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  // set segment as cylinder and set params
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC); // set method for estimating coefficients
  seg.setNormalDistanceWeight(0.1);   // set weight for normal of surface points
  seg.setMaxIterations(5000);
  seg.setDistanceThreshold(0.05); // only points that are close to model will be taken into consideration
  seg.setRadiusLimits(0.1, 0.3);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);
  // get cylinder model coefficients; input cloud is points on cylinder, thus, inliers_cylinder is useless
  seg.segment(*inliers_cylinder, *coefficients_cylinder);

  // cylinder equation
  std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // customized
  bcloud::fit::CylinderFit cylinder_fit;
  cylinder_fit.cloud = *cloud;

  cylinder_fit.fit();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}