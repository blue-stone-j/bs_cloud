#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

namespace bcloud
{
namespace fit
{
Eigen::VectorXd fitQuadraticSurface(const std::vector<pcl::PointXYZ> &neighborhood);
int caller(std::string pcd_path);
} // namespace fit
} // namespace bcloud