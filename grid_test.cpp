#include "grid.h"
#include <gtest/gtest.h>

namespace {

// The fixture for testing class grid.
class GridTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  GridTest() {
    // You can do set-up work for each test here.
  }

  virtual ~GridTest() {
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
TEST_F(GridTest, Basic) {
  Grid grid(3,3);

  int loops = 0;
  for (auto& p : grid) {
    ++loops;
    EXPECT_LE(p.x, 3);
    EXPECT_LE(p.y, 3);
  }
  EXPECT_EQ(9, loops);
}

TEST_F(GridTest, Mark) {
  Grid grid(3,3);

  grid.mark(1,1);
  grid.mark(3,3); // Should be no-op.
  int loops = 0;
  for (auto& p : grid) {
    ++loops;
    EXPECT_FALSE(p.x == 1 && p.y == 1);
  }
  EXPECT_EQ(8, loops);
}

TEST_F(GridTest, GroupMark) {
  Grid grid(3,3);

  grid.groupmark(1,1);
  int loops = 0;
  for (auto& p : grid) {
    ++loops;
    EXPECT_EQ(-1, p.x);
    EXPECT_EQ(-1, p.y);
  }
  EXPECT_EQ(0, loops);
}

TEST_F(GridTest, GroupMark1) {
  Grid grid(3,3);

  grid.groupmark(0,0);
  int loops = 0;
  for (auto& p : grid) {
    ++loops;
    EXPECT_TRUE(p.x >= 2 || p.y >= 2);
  }
  EXPECT_EQ(5, loops);
}

TEST_F(GridTest, MarkCount) {
  Grid grid(3,3);

  for (auto& p : Region(Pos(-1,-1), Pos(3,3))) {
    int count = grid.mark(p.x, p.y);
    if (p.x < 0 || p.y < 0) {
      EXPECT_EQ(0, count) << p.x << "," << p.y;
    } else if (p.x >= 3 || p.y >= 3) {
      EXPECT_EQ(0, count) << p.x << "," << p.y;
    } else {
      EXPECT_EQ(1, count) << p.x << "," << p.y;
    }
  }

  for (auto& p : Region(Pos(-1,-1), Pos(3,3))) {
    int count = grid.mark(p.x, p.y);
    EXPECT_EQ(0, count) << p.x << "," << p.y;
  }
}
TEST_F(GridTest, GroupCount) {
  Grid grid(3,3);
  EXPECT_EQ(9, grid.groupmark(1,1));
  EXPECT_EQ(0, grid.groupmark(1,1));
}


TEST_F(GridTest, GroupCount1) {
  Grid grid(3,3);

  int count = grid.groupmark(0,0);
  EXPECT_EQ(4, count);
  int marks = 0;
  for (auto& p : Region(2,2)) {
    if (p.x < 2 && p.y < 2) {
      EXPECT_FALSE(grid.isAvailable(p.x,p.y)) << p.x << "," << p.y;
      marks++;
    } else {
      EXPECT_TRUE(grid.isAvailable(p.x,p.y)) << p.x << "," << p.y;
    }
  }
  EXPECT_EQ(count, marks);

  count = grid.groupmark(0,0);
  EXPECT_EQ(0, count);

  count = grid.groupmark(1,1);
  EXPECT_EQ(5, count);
}

TEST_F(GridTest, Iter1) {
  Grid grid(10,10);

  int count = grid.groupmark(2,4);
  count += grid.groupmark(7,3);
  EXPECT_EQ(18, count);

  Pos last(-1,-1);
  int loops;
  for (auto& p : grid) {
    EXPECT_TRUE(p != last);
    last = p;
  }
  EXPECT_EQ(0, loops);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
