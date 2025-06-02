
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
};
} // namespace fit

} // namespace bcloud

#endif