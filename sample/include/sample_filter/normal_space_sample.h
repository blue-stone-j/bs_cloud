#ifndef NORMAL_SPACE_SAMPLE_H
#define NORMAL_SPACE_SAMPLE_H


#include "interface.h"

namespace SampleFilter
{
struct NormalSpaceSampleParams
{
  float ratio  = 0.2;
  float minDis = 5, maxDis = 100, thrDis = 3;
};

class NormalSpaceSample : public Interface
{};
} // namespace SampleFilter

#endif