#include <gtest/gtest.h>

#include "fit/surface/quadratic_surface.h"

TEST(surface_test, test1)
{
  bcloud::fit::caller("./assets/cloud/flat.pcd");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}