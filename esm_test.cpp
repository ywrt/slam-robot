/*
 * esm_test.cpp
 *
 *  Created on: 10/02/2013
 *      Author: moreil
 */


#include "esm.h"
#include <gtest/gtest.h>

namespace {

// The fixture for testing class grid.
class EsmTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  EsmTest() {
    // You can do set-up work for each test here.
  }

  virtual ~EsmTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).

  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }
};

// Tests that the grid::Bar() method does Abc.
TEST_F(EsmTest, Basic) {
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



