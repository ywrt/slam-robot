/*
 * planner_test.cpp
 *
 *  Created on: Jan 1, 2013
 *      Author: michael
 */

#include <gtest/gtest.h>

#include "planner.h"

namespace {

// The fixture for testing class grid.
class PlannerTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  PlannerTest() {
    // You can do set-up work for each test here.
  }

  virtual ~PlannerTest() {
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

TEST_F(PlannerTest, Rotate) {
  Vector2d v;
  v << 2, 1;

  v = Planner::rotate_left(v);
  EXPECT_EQ(-1, v(0));
  EXPECT_EQ(2, v(1));

  v = Planner::rotate_left(v);
  EXPECT_EQ(-2, v(0));
  EXPECT_EQ(-1, v(1));

  v = Planner::rotate_left(v);
  EXPECT_EQ(1, v(0));
  EXPECT_EQ(-2, v(1));

  v = Planner::rotate_left(v);
  EXPECT_EQ(2, v(0));
  EXPECT_EQ(1, v(1));

  Vector2d r;
  r = Planner::rotate_left(
          Planner::rotate_right(v));
  EXPECT_EQ(r(0), v(0));
  EXPECT_EQ(r(1), v(1));

  r = Planner::rotate_left(
          Planner::rotate_left(
              Planner::rotate_right(
                  Planner::rotate_right(v))));
  EXPECT_EQ(r(0), v(0));
  EXPECT_EQ(r(1), v(1));

  r = Planner::rotate_right(
          Planner::rotate_right(
              Planner::rotate_right(v)));
  Vector2d r1 = Planner::rotate_left(v);
  EXPECT_EQ(r(0), r1(0));
  EXPECT_EQ(r(1), r1(1));
}

// Tests that the grid::Bar() method does Abc.
TEST_F(PlannerTest, Start) {
  planWithSimpleSetup();
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
