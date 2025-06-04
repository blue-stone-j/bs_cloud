#ifndef CYLINDER_FIT_H
#define CYLINDER_FIT_H

#include "fit/common/interface.h"
namespace bcloud
{
namespace fit
{
/* coefficients of cylinder
   point_on_axis.x, point_on_axis.y, point_on_axis.z
   axis_direction.x, axis_direction.y, axis_direction.z
   radius
 */
class CylinderFit : public FitInterface
{
 public:
  int fit();
};
} // namespace fit

} // namespace bcloud

#endif