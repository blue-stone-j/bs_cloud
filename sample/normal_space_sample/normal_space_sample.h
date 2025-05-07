#ifndef NORMAL_SPACE_SAMPLE_H
#define NORMAL_SPACE_SAMPLE_H

#include "sample/common/interface.h"

namespace Sample
{
struct NormalSpaceSampleParams
{
  float ratio  = 0.2;
  float minDis = 5, maxDis = 100, thrDis = 3;
};

class NormalSpaceSample : public Interface
{};
} // namespace Sample

#endif