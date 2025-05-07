#include <gtest/gtest.h>

#include <pcl/filters/voxel_grid.h>
#include <unordered_map>


#include "sample/grid_sample/grid_sample.h"

TEST(grid_sample_test, Cloud)
{
}

// 定义一个结构体，用于Eigen::Vector3i的哈希函数
struct Vector3iHash
{
  std::size_t operator()(const Eigen::Vector3i &v) const
  {
    // 使用一个基础的哈希函数来合并三个整数的哈希值
    std::size_t hx = std::hash<int>()(v[0]);
    std::size_t hy = std::hash<int>()(v[1]);
    std::size_t hz = std::hash<int>()(v[2]);
    // 合并哈希值的一种简单方式，可以根据需要选择更复杂的方法
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // 加载点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../assets/cloud/fog.pcd", *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file your_point_cloud_file.pcd \n");
    return (-1);
  }

  // 应用体素网格滤波器
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(1.0f, 1.0f, 1.0f); // 设置体素大小

  // 计算每个体素的索引，并统计每个索引的出现次数
  std::unordered_map<Eigen::Vector3i, std::vector<int>, Vector3iHash> voxel_indices;
  size_t min_points_per_voxel = 40; // 设定的阈值

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    Eigen::Vector3i index = vg.getGridCoordinates(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    voxel_indices[index].push_back(i);
  }

  // 筛选出满足点数条件的体素中的点
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  std::cout << voxel_indices.size() << std::endl;
  for (const auto &kv : voxel_indices)
  {
    std::cout << kv.second.size() << std::endl;
    if (kv.second.size() >= min_points_per_voxel)
    {
      inliers->indices.insert(inliers->indices.end(), kv.second.begin(), kv.second.end());
    }
  }

  // 提取满足条件的点云
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_filtered);

  // 保存过滤后的点云
  pcl::io::savePCDFileBinary("../result/filtered_point_cloud.pcd", *cloud_filtered);

  Sample::GridSample grid;


  return RUN_ALL_TESTS();
}