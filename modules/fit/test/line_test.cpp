#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // 拟合直线
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

TEST(Test, test1)
{
  //-------------------------- 加载点云 --------------------------
  std::string path = "../assets/cloud/line1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(path, *cloud) < 0)
  {
    PCL_ERROR("点云读取失败！\n");
    ASSERT_TRUE(false);
  }
  std::cout << "点云点数为：" << cloud->points.size() << std::endl;
  //-----------------------------拟合直线-----------------------------
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
  ransac.setDistanceThreshold(0.01); // 内点到模型的最大距离
  ransac.setMaxIterations(1000);     // 最大迭代次数
  ransac.computeModel();             // 直线拟合
  //--------------------------根据索引提取内点------------------------
  std::vector<int> inliers;
  ransac.getInliers(inliers);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_line);

  //----------------------------输出模型参数--------------------------
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);
  std::cout << "直线方程为：\n"
            << "   (x - " << coefficients[0] << ") / " << coefficients[3]
            << " = (y - " << coefficients[1] << ") / " << coefficients[4]
            << " = (z - " << coefficients[2] << ") / " << coefficients[5] << std::endl;
}

TEST(Test, test2)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // RANSAC拟合多条直线
  // 内点点云合并
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lines(new pcl::PointCloud<pcl::PointXYZ>());
  while (cloud->size() > 20)
  // 循环条件
  {
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
    ransac.setDistanceThreshold(0.05); // 内点到模型的最大距离
    ransac.setMaxIterations(100);      // 最大迭代次数
    ransac.computeModel();             // 直线拟合
                                       // 根据索引提取内点
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_line);
    // 若内点尺寸过小，不用继续拟合，跳出循环
    if (cloud_line->width * cloud_line->height < 20) { break; }
    *cloud_lines = *cloud_lines + *cloud_line;
    // pcl::io::savePCDFile(path1+ strcount +"_"+ str + ".pcd", *cloud_line);
    // 提取外点
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices);
    inliersPtr->indices = inliers;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliersPtr);
    extract.setNegative(true); // 设置为true表示提取外点
    extract.filter(*outliers);
    // pcl::io::savePCDFile("C:/pclpoint/data/cp1_lineout"+str+".pcd", *outliers);
    // cout << outliers->size() << endl;cloud->clear();*cloud = *outliers;}
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}