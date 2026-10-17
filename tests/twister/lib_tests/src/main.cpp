// SPDX-License-Identifier: GPL-3.0-or-later
// Entry point that runs the GoogleTest suite and ends the simulation.
#include <cstdlib>

#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  std::exit(result);
}
