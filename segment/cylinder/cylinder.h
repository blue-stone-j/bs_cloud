#ifndef CYLINDER_H
#define CYLINDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace bcloud
{
namespace segment
{
struct CylinderParams
{
  CylinderParams( )
  {}
};
struct CylinderModel
{};

class CylinderFilter
{
  CylinderFilter(CylinderParams params = CylinderParams( ));
};
} // namespace segment

} // namespace bcloud


#endif
