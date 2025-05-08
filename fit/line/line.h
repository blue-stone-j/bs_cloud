#ifndef LINE_FIT_H
#define LINE_FIT_H

#include "fit/common/interface.h"

namespace bcloud
{
namespace fit
{
class LineFit : public FitInterface
{
 public:
  int fit();
};
} // namespace fit

} // namespace bcloud

#endif