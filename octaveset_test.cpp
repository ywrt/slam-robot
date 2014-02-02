#include "octaveset.h"
#include <gtest/gtest.h>

namespace {

// The fixture for testing class OctaveSet.
class OctaveSetTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  OctaveSetTest() {
    // You can do set-up work for each test here.
  }

  virtual ~OctaveSetTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
    uint8_t data[height_ * width_];
    for (int i = 0; i < height_ * width_; ++i)
      data[i] = i;
    o_.FillOctaves(data, width_, height_);
  }

  void Zero() {
    uint8_t data[height_ * width_];
    for (int i = 0; i < height_ * width_; ++i)
      data[i] = 0;
    o_.FillOctaves(data, width_, height_);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for OctaveSet.
  OctaveSet o_;
  const int width_ = 136;
  const int height_ = 155;
};

// Tests that the OctaveSet::Bar() method does Abc.
TEST_F(OctaveSetTest, Fill) {
}

TEST_F(OctaveSetTest, SearchBestCorner) {
  uint8_t data[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i)
    data[i] = 0;

  int offset = (height_ / 2) * width_ + width_ / 2;
  data[offset] = 250;
  data[offset - 1] = 250;
  data[offset + 1] = 250;
  data[offset - 2] = 250;
  data[offset + 2] = 250;
  data[offset - width_] = 250;
  data[offset + width_] = 250;

  o_.FillOctaves(data, width_, height_);

  FPos fp = o_.SearchBestCorner(FRegion(FPos(0,0), FPos(1,1)), 0);
  EXPECT_FALSE(fp.isInvalid());

  EXPECT_NEAR(width_ / 2, fp.x * width_, 0.02);
  EXPECT_NEAR(height_ / 2, fp.y * height_, 0.02);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
