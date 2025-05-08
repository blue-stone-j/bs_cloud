#include "fit/plane/plane.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace bcloud
{
namespace fit
{
int PlaneFit::fit()
{
  { // PCA(Principal components analysis)
    int num_points = cloud.size();

    Eigen::MatrixXd points(num_points, 3);
    for (int i = 0; i < num_points; i++)
    {
      points(i, 0) = cloud[i].x;
      points(i, 1) = cloud[i].y;
      points(i, 2) = cloud[i].z;
    }

    // gravity
    Eigen::Vector3d centroid = points.colwise().mean();

    // decentralize
    Eigen::MatrixXd demeaned_points = points.rowwise() - centroid.transpose();

    // covariance
    Eigen::Matrix3d cov_matrix = demeaned_points.transpose() * demeaned_points / double(num_points);

    // eigen values and eigen vectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov_matrix);

    // eigen vector corresponding to minimal eigen value
    Eigen::Vector3d normal_vector = es.eigenvectors().col(0);

    // d of plane equation
    double d = -normal_vector.dot(centroid);

    std::cout << "The plane equation is " << normal_vector(0) << "x + " << normal_vector(1) << "y + " << normal_vector(2) << "z + " << d << " = 0" << std::endl;
  }

  { // least square
    int num_points = cloud.size();

    Eigen::MatrixXd points(num_points, 3);
    for (int i = 0; i < num_points; i++)
    {
      points(i, 0) = cloud[i].x;
      points(i, 1) = cloud[i].y;
      points(i, 2) = cloud[i].z;
    }
    // construct matrix A and vector B
    Eigen::MatrixXd A(num_points, 3);
    A.col(0) = points.col(0);                     // x
    A.col(1) = points.col(1);                     // y
    A.col(2) = Eigen::VectorXd::Ones(num_points); // constant

    Eigen::VectorXd B = points.col(2); // z

    // solve by least square
    Eigen::VectorXd coeff = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    double norm = std::sqrt(1 + coeff(0) * coeff(0) + coeff(1) * coeff(1));

    std::cout << "The plane equation is " << -coeff(0) / norm << "x + " << -coeff(1) / norm << "y + " << 1 / norm << "z + " << -coeff(2) / norm << " = 0" << std::endl;
  }

  return 0;
}
} // namespace fit

} // namespace bcloud
