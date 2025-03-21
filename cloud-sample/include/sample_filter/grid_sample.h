#ifndef GRID_SAMPLE_H
#define GRID_SAMPLE_H

#include "interface.h"

namespace SampleFilter
{
struct GridSampleParams
{
  float ratio  = 0.2;
  float minDis = 5, maxDis = 100, thrDis = 3;
};

class GridSample : public Interface
{
 public:
  GridSampleParams params;
  GridSample() {}
  GridSample(GridSampleParams params);
  int sample_filter();
};

} // namespace SampleFilter

#endif