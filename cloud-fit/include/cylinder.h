#ifndef CYLINDER_FIT_H
#define CYLINDER_FIT_H

#include "interface.h"

/* coefficients of cylinder
   point_on_axis.x, point_on_axis.y, point_on_axis.z
   axis_direction.x, axis_direction.y, axis_direction.z
   radius
 */
class CylinderFit : public Interface
{
 public:
  int fit();
};

#endif