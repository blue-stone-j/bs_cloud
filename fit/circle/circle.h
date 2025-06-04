
#ifndef FIT_CIRCLE_H
#define FIT_CIRCLE_H

#include "fit/common/interface.h"

namespace bcloud
{
namespace fit
{
// 2D Circle fitting class
class CircleFit : public FitInterface
{
 public:
  CircleFit();
  // Fit a circle to the given points
  int fit();
  Eigen::Vector2f fitCircleKnownRadius2D(const std::vector<Eigen::Vector2f> &points, float radius);
};
} // namespace fit

} // namespace bcloud

#endif