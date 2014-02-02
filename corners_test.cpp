#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <cstdlib>

#include "octave.h"
#include "corners.h"
#include "faster.h"

#include <gtest/gtest.h>

namespace {

using namespace cv;

// The fixture for testing class Corners.
class CornersTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  CornersTest() : width_(0), height_(0) {
    // You can do set-up work for each test here.
  }

  virtual ~CornersTest() {
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

  void Zero() {
    uint8_t data[height_ * width_];
    for (int i = 0; i < height_ * width_; ++i)
      data[i] = 0;
    o_.copy(data, width_, height_);
  }

  int width_;
  int height_;
  Octave o_;
  Octave o1_;
};

TEST_F(CornersTest, NonMax_SingletonIsNoOp) {
  vector<faster::Corner> corners;
  corners.push_back({10, 10, 100});

  SupressNonMax(5, &corners);
  EXPECT_EQ(100, corners[0].score);
}

TEST_F(CornersTest, NonMax_NonIntersectingIsNoOp) {
  vector<faster::Corner> corners;
  corners.push_back({10, 10, 100});
  corners.push_back({4, 4, 200});
  SupressNonMax(5, &corners);
  EXPECT_EQ(100, corners[0].score);
  EXPECT_EQ(200, corners[1].score);
}

TEST_F(CornersTest, NonMax_OverLapWillSupress) {
  vector<faster::Corner> corners(2);

  for (int i = 5 ; i <= 15; ++i) {
    for (int j = 5 ; j <= 15; ++j) {
      if (i == 10 && j == 10) continue;
      corners[0] = {10, 10, 100};
      corners[1] = {i, j, 50};
      sort(corners.begin(), corners.end(), [](const faster::Corner& a, const faster::Corner& b) -> bool {
            return a.y < b.y;
          });
      SupressNonMax(5, &corners);
      int idx = 0;
      if (corners[0].score != 100)
        ++idx;
      EXPECT_EQ(100, corners[idx].score);
      EXPECT_EQ(-1, corners[idx ^ 1].score) << i << " " << j << "\n";
    }
  }
}

TEST_F(CornersTest, NonMax_OutSideRangeWontSupress) {
  vector<faster::Corner> corners(2);

  for (int i = 0 ; i <= 20; ++i) {
    for (int j = 0 ; j <= 20; ++j) {
      if (i >= 5 && i <= 15 && j >= 5 && j <= 15)
        continue;
      corners[0] = {10, 10, 100};
      corners[1] = {i, j, 50};
      sort(corners.begin(), corners.end(), [](const faster::Corner& a, const faster::Corner& b) -> bool {
            return a.y < b.y;
          });
      SupressNonMax(5, &corners);
      int idx = 0;
      if (corners[0].score != 100)
        ++idx;
      EXPECT_EQ(100, corners[idx].score);
      EXPECT_EQ(50, corners[idx ^ 1].score) << i << " " << j << "\n";
    }
  }
}

TEST_F(CornersTest, NonMax_DenseSupression) {
  vector<faster::Corner> corners(2);

  for (int i = 0 ; i <= 20; ++i) {
    for (int j = 0 ; j <= 20; ++j) {
      if (i == 10 && j == 10)
        corners.push_back({i, j, 100});
      else
        corners.push_back({i, j, 50});
    }
  }
  sort(corners.begin(), corners.end(), [](const faster::Corner& a, const faster::Corner& b) -> bool {
        return a.y < b.y;
      });
  SupressNonMax(5, &corners);

  vector<faster::Corner> result;
  for (auto& c : corners) {
    if (c.score < 0)
      continue;
    result.push_back(c);
  }

  for (int i = 0; i < result.size(); ++i) {
    for (int j = i + 1; j < result.size(); ++j) {
      int dx = abs(result[i].x - result[j].x);
      int dy = abs(result[i].y - result[j].y);
      EXPECT_TRUE(dx > 5 || dy > 5) << "i:" << i << " j:" << j << " i.x:" << result[i].x << " j.x:" << result[j].x;
    }
  }
}

TEST_F(CornersTest, NonMax_AscendingDenseSupression) {
  vector<faster::Corner> corners(2);

  for (int i = 0 ; i <= 20; ++i) {
    for (int j = 0 ; j <= 20; ++j) {
      corners.push_back({i, j, i + j});
    }
  }
  sort(corners.begin(), corners.end(), [](const faster::Corner& a, const faster::Corner& b) -> bool {
        return a.y < b.y;
      });
  SupressNonMax(5, &corners);

  vector<faster::Corner> result;
  for (auto& c : corners) {
    if (c.score < 0)
      continue;
    result.push_back(c);
  }

  for (int i = 0; i < result.size(); ++i) {
    for (int j = i + 1; j < result.size(); ++j) {
      int dx = abs(result[i].x - result[j].x);
      int dy = abs(result[i].y - result[j].y);
      EXPECT_TRUE(dx > 5 || dy > 5) << "i:" << i << " j:" << j << " i.x:" << result[i].x << " j.x:" << result[j].x;
    }
  }
}

TEST_F(CornersTest, Image) {
  Mat mat;
  mat = imread("data/Wood1/view1.png", CV_LOAD_IMAGE_GRAYSCALE);

  width_ = mat.cols;
  height_ = mat.rows;

  uint8_t data[height_ * width_];
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      data[x + y * width_] = mat.data[y * mat.step[0] + x];
    }
  }

  o_.copy(data, width_, height_);

  auto corners = FindCorners(o_);
#if 1
  for (auto&c : corners) {
    line(mat, Point(c.x, c.y - 3), Point(c.x, c.y + 3), Scalar(0,0,0), 1, 8);
    line(mat, Point(c.x - 3, c.y), Point(c.x + 3, c.y), Scalar(0,0,0), 1, 8);
  }

  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", mat);                   // Show our image inside it.
  //waitKey(0);
#endif
  return;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      mat.data[y * mat.step[0] + x] = o_.pixel(x, y);
    }
  }

  mat = imread("data/Wood1/view5.png", CV_LOAD_IMAGE_GRAYSCALE);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      data[x + y * width_] = mat.data[y * mat.step[0] + x];
    }
  }
  o1_.copy(data, width_, height_);
  o1_.Smooth();
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
