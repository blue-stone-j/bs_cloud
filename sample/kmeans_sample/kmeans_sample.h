#ifndef KMEANS_SAMPLE_H
#define KMEANS_SAMPLE_H

#include "sample/common/interface.h"

namespace Sample
{
struct KMeansSampleParams
{
  float ratio  = 0.2;
  float minDis = 5, maxDis = 100, thrDis = 3;
};

class KMeansSample : public Interface
{};
} // namespace Sample

#endif