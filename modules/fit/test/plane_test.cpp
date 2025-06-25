#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

#include "fit/plane/plane.h"

TEST(plane_test, pcl)
{
  std::string path = "../assets/cloud/plane1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file.pcd \n");
    ASSERT_TRUE(false);
  }

  // pcl
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices inliers;
  inliers.indices.clear();
  pcl::ModelCoefficients coefficients;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(cloud);
  seg.segment(inliers, coefficients);

  // plane equation
  std::cout << "The plane equation is " << coefficients.values[0] << "x + "
            << coefficients.values[1] << "y + " << coefficients.values[2] << "z + "
            << coefficients.values[3] << " = 0" << std::endl;


  // customized
  bcloud::fit::PlaneFit plane_fit;
  plane_fit.cloud = *cloud;

  plane_fit.fit();
}

TEST(plane_test, custom)
{
  std::string path = "../assets/cloud/plane1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file point_cloud_file.pcd \n");
    ASSERT_TRUE(false);
  }

  // 内点点云合并
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planes(new pcl::PointCloud<pcl::PointXYZ>());
  while (cloud->size() > 100) // 循环条件
  {
    //--------------------------RANSAC拟合平面--------------------------
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
    ransac.setDistanceThreshold(0.01); // 设置距离阈值，与平面距离小于0.1的点作为内点
    ransac.setMaxIterations(100);      // 最大迭代次数
    ransac.computeModel();             // 执行模型估计

    //-------------------------根据索引提取内点--------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inplanes;                                          // 存储内点索引的容器
    ransac.getInliers(inplanes);                                        // 提取内点索引
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inplanes, *cloud_plane); // 若内点尺寸过小，不用继续拟合，跳出循环
    if (cloud_plane->width * cloud_plane->height < 100) { break; }
    *cloud_planes = *cloud_planes + *cloud_plane;
    // 提取外点
    pcl::PointCloud<pcl::PointXYZ>::Ptr outplanes(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inplanePtr(new pcl::PointIndices);
    inplanePtr->indices = inplanes;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inplanePtr);
    extract.setNegative(true); // 设置为true表示提取外点
    extract.filter(*outplanes);
    // pcl::io::savePCDFile("C:/pclpoint/data/cp1_lineout"+str+".pcd", *outliers);
    // cout << outliers->size() << endl;cloud->clear();*cloud = *outplanes;
    //----------------------------输出模型参数---------------------------/*
    Eigen::VectorXf coefficient;
    ransac.getModelCoefficients(coefficient);
    std::cout << "平面方程为：\n"
              << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << " = 0" << std::endl;
  }
  ///返回最终的拟合结果点云return cloud_planes;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}