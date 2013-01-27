/*
 * histogram_test.cpp
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#include <gtest/gtest.h>

#include "histogram.h"

namespace {

// The fixture for testing class grid.
class HistogramTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  HistogramTest() {
    // You can do set-up work for each test here.
  }

  virtual ~HistogramTest() {
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
TEST_F(HistogramTest, Basic) {
  Histogram h(2);
  EXPECT_EQ(0, h.bucket(0));
  EXPECT_EQ(0, h.bucket(1));

  h.add(0);
  EXPECT_EQ(1, h.bucket(0));

  h.add(1);
  EXPECT_EQ(1, h.bucket(0));
  EXPECT_EQ(1, h.bucket(1));

  h.add(2);
  EXPECT_EQ(1, h.bucket(0));
  EXPECT_EQ(2, h.bucket(1));
}

TEST_F(HistogramTest, Scale) {
  Histogram h(2, 2);
  EXPECT_EQ(0, h.bucket(0));
  EXPECT_EQ(0, h.bucket(1));

  h.add(0);
  EXPECT_EQ(1, h.bucket(0));

  h.add(1);
  EXPECT_EQ(2, h.bucket(0));
  EXPECT_EQ(0, h.bucket(1));

  h.add(2);
  EXPECT_EQ(2, h.bucket(0));
  EXPECT_EQ(1, h.bucket(1));
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
