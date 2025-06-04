
#include "fit/circle/circle.h"

namespace bcloud
{
namespace fit
{
CircleFit::CircleFit()
{
  model_coefficients.resize(3);
}

int CircleFit::fit()
{
  float sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0, sum_xy = 0, sum_r = 0;
  int N = cloud.size();

  for (const auto &point : cloud)
  {
    float x = point.x, y = point.y;
    float r2 = x * x + y * y;

    sum_x += x;
    sum_y += y;
    sum_x2 += x * x;
    sum_y2 += y * y;
    sum_xy += x * y;
    sum_r += r2;
  }

  Eigen::Matrix3f A;
  Eigen::Vector3f B;

  A << sum_x2, sum_xy, sum_x,
      sum_xy, sum_y2, sum_y,
      sum_x, sum_y, N;

  B << -sum_r, -sum_r, -sum_x2 - sum_y2;

  Eigen::Vector3f solution = A.colPivHouseholderQr().solve(B);
  float cx                 = -0.5f * solution(0);
  float cy                 = -0.5f * solution(1);
  float radius             = std::sqrt(cx * cx + cy * cy - solution(2));
  model_coefficients[0]    = cx;     // Circle center x
  model_coefficients[1]    = cy;     // Circle center y
  model_coefficients[2]    = radius; // Circle radius
  std::cout << "Circle coefficients: " << model_coefficients[0] << ", "
            << model_coefficients[1] << ", " << model_coefficients[2] << std::endl;

  return 0;
}

Eigen::Vector2f CircleFit::fitCircleKnownRadius2D(const std::vector<Eigen::Vector2f> &points, float radius)
{
  // If there are no points, return trivial center (0,0)
  if (points.empty())
  {
    return Eigen::Vector2f::Zero();
  }

  // Initialize center (cx, cy) to the centroid of the data
  Eigen::Vector2f centroid(0.0f, 0.0f);
  for (const auto &p : points)
  {
    centroid += p;
  }
  centroid /= static_cast<float>(points.size());

  Eigen::Vector2f center = centroid;

  // Simple parameters for iterative updates
  float learningRate = 0.01f;
  int maxIters       = 100;
  float epsilon      = 1e-6f;

  for (int iter = 0; iter < maxIters; ++iter)
  {
    Eigen::Vector2f grad(0.0f, 0.0f);

    // Compute gradient of the objective: sum( (|p - c| - R)^2 )
    // d/d(cx, cy) = 2 * sum( (|p - c| - R) * ( (|p - c| - R)' ) )
    // but more directly: partial derivative w.r.t c = 2 * (|p - c| - R) * (c - p)/|c - p|
    // We accumulate these for all points.
    for (const auto &p : points)
    {
      Eigen::Vector2f diff = center - p;
      float dist           = diff.norm();
      if (dist < 1e-9f)
      {
        continue; // Avoid singularities
      }
      float residual = (dist - radius);
      grad += (residual * diff / dist);
    }

    // Update center
    grad *= (2.0f); // factor 2 from the derivative of squared residual
    Eigen::Vector2f update = learningRate * grad;
    if (update.norm() < epsilon)
    {
      break; // Converged
    }
    center -= update;
  }

  return center;
}

} // namespace fit

} // namespace bcloud