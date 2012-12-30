/*
 * localmap_test.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

#include <gtest/gtest.h>

#include "localmap.h"

namespace {

// The fixture for testing class grid.
class LocalMapTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  LocalMapTest() {
    // You can do set-up work for each test here.
  }

  virtual ~LocalMapTest() {
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
TEST_F(LocalMapTest, Abc) {
  EXPECT_TRUE(true);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
