#include "octave.h"
#include <gtest/gtest.h>

namespace {

// The fixture for testing class Octave.
class OctaveTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  OctaveTest() {
    // You can do set-up work for each test here.
  }

  virtual ~OctaveTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).

    uint8_t data[rows_ * cols_];
    for (int i = 0; i < rows_ * cols_; ++i)
      data[i] = i;
    o_.copy(data, cols_, rows_);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }
  const int rows_ = 33;
  const int cols_ = 17;
  Octave o_;
};

// Tests that the Octave::Bar() method does Abc.
TEST_F(OctaveTest, Copy) {
  // fill from 11 x 13 data.
  const int rows = 11;
  const int cols = 13;
  uint8_t data[rows * cols];
  for (int i = 0; i < rows * cols; ++i)
    data[i] = i;
  o_.copy(data, cols, rows);

  EXPECT_EQ(o_.width_, cols);
  EXPECT_EQ(o_.height_, rows);

  for (int row = 0; row < 8; ++row) {
    for (int col = 0; col < 8; ++col) {
      int offset = col + row * cols;
      int index = col + row * 8;
      EXPECT_EQ(offset, o_.pixel(col, row));
    }
  }
}

TEST_F(OctaveTest, Fill) {
  Octave o1;
  o1.fill(o_);

  EXPECT_EQ(o1.width_, o_.width_ / 2);
  EXPECT_EQ(o1.height_, o_.height_ / 2);
  EXPECT_EQ(((0+1)*2+(0+1)*17*2)/4, o1.pixel(0,0));
  EXPECT_EQ(((2+3)*2 + (2+3)*17*2)/4, o1.pixel(1,1));
  EXPECT_EQ(((6+7)*2 + (4+5)*17*2)/4, o1.pixel(3,2));
}

TEST_F(OctaveTest, Score) {
  uint8_t patch[64];
  for (auto& p : Region(8,8)) {
    patch[p.x + p.y * 8] = o_.pixel(p);
  }

  int score = o_.Score(patch, Pos(4,4));
  EXPECT_EQ(0, score);
}

TEST_F(OctaveTest, Search) {
  uint8_t patch[64];
  for (auto& p : Region(8,8)) {
    patch[p.x + p.y * 8] = o_.pixel(p + Pos(2,4));
  }

  // Fail to search in the region with an exact match.
  int score = 0;
  FPos fp;
  fp = o_.searchPosition(FPos(0, 0), patch, 6, &score);
  EXPECT_LT(0, score);
  ASSERT_NEAR(fp.x * cols_, 6, 0.1);
  ASSERT_NEAR(fp.y * rows_, 6, 0.1);
  EXPECT_GE(0, 6. - fp.x * cols_);
  EXPECT_GE(0, 6. - fp.y * rows_);

  // Search over a large region that includes the exact match.
  fp = o_.searchPosition(FPos(0, 0), patch, 9, &score);
  EXPECT_EQ(0, score);
  ASSERT_NEAR(fp.x * cols_, 6, 0.1);
  ASSERT_NEAR(fp.y * rows_, 8, 0.1);

}



}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
