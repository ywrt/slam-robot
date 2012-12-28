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
    const int rows = 33;
    const int cols = 17;
    uint8_t data[rows * cols];
    for (int i = 0; i < rows * cols; ++i)
      data[i] = i;
    o.copy(data, cols, rows);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }
  Octave o;
};

// Tests that the Octave::Bar() method does Abc.
TEST_F(OctaveTest, Copy) {
  // fill from 11 x 13 data.
  const int rows = 11;
  const int cols = 13;
  uint8_t data[rows * cols];
  for (int i = 0; i < rows * cols; ++i)
    data[i] = i;
  o.copy(data, cols, rows);

  EXPECT_EQ(o.width_, cols);
  EXPECT_EQ(o.height_, rows);

  for (int row = 0; row < 8; ++row) {
    for (int col = 0; col < 8; ++col) {
      int offset = col + row * cols;
      int index = col + row * 8;
      EXPECT_EQ(offset, o.pixel(col, row));
    }
  }
}

TEST_F(OctaveTest, Fill) {
  Octave o1;
  o1.fill(o);

  EXPECT_EQ(o1.width_, o.width_ / 2);
  EXPECT_EQ(o1.height_, o.height_ / 2);
  EXPECT_EQ(((0+1)*2+(0+1)*17*2)/4, o1.pixel(0,0));
  EXPECT_EQ(((2+3)*2 + (2+3)*17*2)/4, o1.pixel(1,1));
  EXPECT_EQ(((6+7)*2 + (4+5)*17*2)/4, o1.pixel(3,2));
}


// Tests that Octave does Xyz.
TEST_F(OctaveTest, DoesXyz) {
  // Exercises the Xyz feature of Octave.
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
