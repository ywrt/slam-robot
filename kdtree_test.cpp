#include <vector>

#include <gtest/gtest.h>

#include "kdtree.h"

namespace {

// The fixture for testing class KDTree.
class KDTreeTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  KDTreeTest() {
    // You can do set-up work for each test here.
  }

  virtual ~KDTreeTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

};

TEST_F(KDTreeTest, CanDo1DSearch) {
  std::vector<double> data {1,2,3,4};
  KDTree<double, 1> t(data.begin(), data.end());

  int count = 0;
  double sum = 0;
  t.search(2, 3, [&count, &sum](double d) -> void { count++; sum += d; } );

  EXPECT_EQ(2, count);
  EXPECT_EQ(5, sum);
  
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
