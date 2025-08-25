#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

#include "fit/surface/quadratic_surface.h"

namespace bcloud
{
namespace fit
{
// Function to fit a quadratic surface
Eigen::VectorXd fitQuadraticSurface(const std::vector<pcl::PointXYZ> &neighborhood)
{
  // Number of points
  int n = neighborhood.size();

  // Construct the design matrix A and vector b
  Eigen::MatrixXd A(n, 6);
  Eigen::VectorXd b(n);

  for (int i = 0; i < n; ++i)
  {
    double x = neighborhood[i].x;
    double y = neighborhood[i].y;
    double z = neighborhood[i].z;

    A(i, 0) = x * x;
    A(i, 1) = y * y;
    A(i, 2) = x * y;
    A(i, 3) = x;
    A(i, 4) = y;
    A(i, 5) = 1.0;

    b(i) = z;
  }

  // Solve the normal equation A^T * A * coeffs = A^T * b
  Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

  return coeffs;
}

int caller(std::string pcd_path)
{
  // Load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file pointcloud.pcd \n");
    return (-1);
  }

  // Define the search point
  pcl::PointXYZ searchPoint = cloud->points[0]; // Example: first point in the cloud

  // KdTree for neighborhood search
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  // Define search radius or number of neighbors
  float radius = 0.1;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  // Perform radius search
  if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    // Extract neighborhood points
    std::vector<pcl::PointXYZ> neighborhood;
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      neighborhood.push_back(cloud->points[pointIdxRadiusSearch[i]]);
    }

    // Fit quadratic surface
    Eigen::VectorXd coefficients = fitQuadraticSurface(neighborhood);
    std::cout << "Coefficients: " << coefficients.transpose() << std::endl;
  }
  else
  {
    std::cout << "No neighbors found within the radius." << std::endl;
  }

  return 0;
}

} // namespace fit
} // namespace bcloud