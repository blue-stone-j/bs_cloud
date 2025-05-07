#ifndef LINE_FIT_H
#define LINE_FIT_H

#include "fit/common/interface.h"

class LineFit : public FitInterface
{
 public:
  int fit();
};

#endif