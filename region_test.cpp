#include "region.h"
#include <gtest/gtest.h>

namespace {

// The fixture for testing class region.
class regionTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  regionTest() {
    // You can do set-up work for each test here.
  }

  virtual ~regionTest() {
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

// Tests that the region::Bar() method does Abc.
TEST_F(regionTest, Basic) {
  Pos p1(1,2);
  Pos p2(4,3);

  Region r(p1, p2);
  EXPECT_EQ(1, r.ll.x);
  EXPECT_EQ(2, r.ll.y);
  EXPECT_EQ(4, r.ur.x);
  EXPECT_EQ(3, r.ur.y);
}

TEST_F(regionTest, Iterate) {
  Pos p1(1,2);
  Pos p2(4,3);

  Region r(p1, p2);
  int x = 1;
  int y = 2;
  for (auto& p : r) {
    EXPECT_EQ(p.x, x);
    EXPECT_EQ(p.y, y);

    x++;
    if (x > 4) {
      x = 1;
      ++y;
    }
  }
}

TEST_F(regionTest, NullIterate) {
  Pos p1(4,3);
  Pos p2(1,2);

  Region r(p1, p2);
  int loops = 0;
  for (auto& p : r) {
    EXPECT_LE(0, p.x);
    EXPECT_LE(0, p.y);
    ++loops;
  }
  EXPECT_EQ(0, loops);
}

TEST_F(regionTest, SingleIterate) {
  Pos p1(0,0);
  Pos p2(0,0);

  Region r(p1, p2);
  int loops = 0;
  for (auto& p : r) {
    EXPECT_EQ(0, p.x);
    EXPECT_EQ(0, p.y);
    ++loops;
  }
  EXPECT_EQ(1, loops);
}

TEST_F(regionTest, ThinYIterate) {
  Pos p1(0,0);
  Pos p2(0,3);

  Region r(p1, p2);
  int loops = 0;
  for (auto& p : r) {
    EXPECT_EQ(0, p.x);
    ++loops;
  }
  EXPECT_EQ(4, loops);
}

TEST_F(regionTest, ThinXIterate) {
  Pos p1(0,0);
  Pos p2(3,0);

  Region r(p1, p2);
  int loops = 0;
  for (auto& p : r) {
    EXPECT_EQ(0, p.y);
    ++loops;
  }
  EXPECT_EQ(4, loops);
}


TEST_F(regionTest, Const) {
  Region r(3, 2);
  int loops = 0;
  for (auto& p : r) {
    EXPECT_GT(4, p.x);
    EXPECT_GT(3, p.y);
    ++loops;
  }
  EXPECT_EQ(3*4, loops);
}

// Tests that region does Xyz.
TEST_F(regionTest, DoesXyz) {
  // Exercises the Xyz feature of region.
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
