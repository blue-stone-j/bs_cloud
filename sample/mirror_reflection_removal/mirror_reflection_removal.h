#ifndef MIRROR_REFLECTION_REMOVAL_H
#define MIRROR_REFLECTION_REMOVAL_H

#include "sample/common/interface.h"

namespace Sample
{
struct MirrorReflectionRemovalParams
{
  float ratio  = 0.2;
  float minDis = 5, maxDis = 100, thrDis = 3;
};

class MirrorReflectionRemoval : public Interface
{};
} // namespace Sample


#endif
