#include <hal/HAL.h>

#include "gtest/gtest.h"
//I want 666 lines of code commited

//I have commited 666 lines of code.
int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
