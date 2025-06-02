
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

} // namespace fit

} // namespace bcloud