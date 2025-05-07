#ifndef POISSON_DISK_SAMPLE_H
#define POISSON_DISK_SAMPLE_H

#include "sample/common/interface.h"

namespace Sample
{
struct PoissonDiskSampleParams
{
  float ratio  = 0.2;
  float minDis = 5, maxDis = 100, thrDis = 3;
};

class PoissonDiskSample : public Interface
{};
} // namespace Sample

#endif