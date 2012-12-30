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
    uint8_t data[height_ * width_];
    for (int i = 0; i < height_ * width_; ++i)
      data[i] = i;
    o_.copy(data, width_, height_);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  void Zero() {
    uint8_t data[height_ * width_];
    for (int i = 0; i < height_ * width_; ++i)
      data[i] = 0;
    o_.copy(data, width_, height_);
  }

  const int width_ = 17;
  const int height_ = 33;
  Octave o_;
};

TEST_F(OctaveTest, Region) {
  Region r = o_.sub_region(FRegion(FPos(0,0), FPos(1,1)));
  EXPECT_EQ(0, r.ll.x);
  EXPECT_EQ(0, r.ll.y);
  EXPECT_EQ(width_, r.ur.x);
  EXPECT_EQ(height_, r.ur.y);
}

TEST_F(OctaveTest, Region1) {
  Region r = o_.sub_region(FRegion(FPos(0.2, 0.4), FPos(0.6, 0.8)));
  EXPECT_EQ(3, r.ll.x);
  EXPECT_EQ(13, r.ll.y);
  EXPECT_EQ(10, r.ur.x);
  EXPECT_EQ(26, r.ur.y);
}

TEST_F(OctaveTest, Clip) {
  Region r = o_.clipped_region(FRegion(FPos(0,0), FPos(1,1)), 0);
  EXPECT_EQ(0, r.ll.x);
  EXPECT_EQ(0, r.ll.y);
  EXPECT_EQ(width_ - 1, r.ur.x);
  EXPECT_EQ(height_ - 1, r.ur.y);
}

TEST_F(OctaveTest, ClipMargin) {
  Region r = o_.clipped_region(FRegion(FPos(0,0), FPos(1,1)), 3);
  EXPECT_EQ(3, r.ll.x);
  EXPECT_EQ(3, r.ll.y);
  EXPECT_EQ(width_ - 4, r.ur.x);
  EXPECT_EQ(height_ - 4, r.ur.y);
}

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
  fp = o_.SearchPosition(FPos(0, 0), patch, 6, &score);
  EXPECT_LT(0, score);
  ASSERT_NEAR(fp.x * width_, 6, 0.1);
  ASSERT_NEAR(fp.y * height_, 6, 0.1);
  EXPECT_GE(0, 6. - fp.x * width_);
  EXPECT_GE(0, 6. - fp.y * height_);

  // Search over a large region that includes the exact match.
  fp = o_.SearchPosition(FPos(0, 0), patch, 9, &score);
  EXPECT_EQ(0, score);
  ASSERT_NEAR(fp.x * width_, 6, 0.1);
  ASSERT_NEAR(fp.y * height_, 8, 0.1);

}


TEST_F(OctaveTest, ScoreCorner) {
  Zero();
  *o_.pixel_ptr(Pos(7,9)) = 100;

  Pos best_point;
  int best_score = 0;
  for (auto& p : Region(Pos(3,3), Pos(width_ - 4, height_ - 4))) {
    int score = o_.ScoreCorner(p);
    if (score == 0) {
      EXPECT_TRUE(p.x < 5 ||
                  p.x > 9 ||
                  p.y < 7 ||
                  p.y > 11) << p.x << ", " << p.y << "\n";
    } else {
      EXPECT_FALSE(p.x < 5 || p.x > 9) << p.x;
      EXPECT_FALSE(p.y < 7 || p.y > 11) << p.y;
    }
    if (score > best_score) {
      best_score = score;
      best_point = p;
    }
  }
  EXPECT_LE(10000, best_score);
  EXPECT_EQ(7, best_point.x);
  EXPECT_EQ(9, best_point.y);
}



}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
