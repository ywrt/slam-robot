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
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for OctaveSet.
  OctaveSet o;
};

// Tests that the OctaveSet::Bar() method does Abc.
TEST_F(OctaveSetTest, Fill) {
  // fill from 11 x 13 data.
  const int rows = 11;
  const int cols = 13;
  uint8_t data[rows * cols];
  for (int i = 0; i < rows * cols; ++i)
    data[i] = i;
  o.FillOctaves(data, cols, rows);
}

// Tests that OctaveSet does Xyz.
TEST_F(OctaveSetTest, DoesXyz) {
  // Exercises the Xyz feature of OctaveSet.
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
