#ifndef PLANE_FIT_H
#define PLANE_FIT_H

#include "fit/common/interface.h"

namespace bcloud
{
namespace fit
{
/* coefficients of model
   plane: Ax+By+Cz+D=0, where A^2+B^2+C^2=1
 */
class PlaneFit : public FitInterface
{
 public:
  int fit();
};
} // namespace fit

} // namespace bcloud


#endif