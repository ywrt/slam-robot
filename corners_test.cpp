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

  CornersTest() {
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

  Octave OctaveFromMat(const Mat& mat) {
    int width = mat.cols;
    int height = mat.rows;

    uint8_t data[height * width];
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        data[x + y * width] = mat.data[y * mat.step[0] + x];
      }
    }

    Octave o;
    o.copy(data, width, height);
    return o;
  }

};

TEST(CornerListTest, HandlesEmptyList) {
  CornerList list;
  EXPECT_EQ(-1, list.find(Region(Pos(0,0),Pos(1,1))));
}

TEST(CornerListTest, IgnoresBelow) {
  CornerList list;
  list.corners.push_back(Pos(1,1));
  EXPECT_EQ(-1, list.find(Region(Pos(2,2),Pos(3,3))));
}

TEST(CornerListTest, IgnoresAbove) {
  CornerList list;
  list.corners.push_back(Pos(4,4));
  EXPECT_EQ(-1, list.find(Region(Pos(2,2),Pos(3,3))));
}

TEST(CornerListTest, IgnoresToTheLeft) {
  CornerList list;
  list.corners.push_back(Pos(1,2));
  EXPECT_EQ(-1, list.find(Region(Pos(2,2),Pos(3,3))));
}

TEST(CornerListTest, IgnoresToTheRight) {
  CornerList list;
  list.corners.push_back(Pos(4,3));
  EXPECT_EQ(-1, list.find(Region(Pos(2,2),Pos(3,3))));
}

TEST(CornerListTest, FindsInRegion) {
  CornerList list;
  list.corners.push_back(Pos(3,3));
  EXPECT_EQ(0, list.find(Region(Pos(3,3),Pos(3,3))));
}

TEST(CornerListTest, FindsInRegionAndIgnoresOthers) {
  CornerList list;
  list.corners.push_back(Pos(2,2));
  list.corners.push_back(Pos(3,3));
  list.corners.push_back(Pos(4,4));
  EXPECT_EQ(1, list.find(Region(Pos(3,3),Pos(3,3))));
}

TEST(CornerListTest, NextWontAdvanceOutOfRegion) {
  CornerList list;
  list.corners.push_back(Pos(2,2));
  list.corners.push_back(Pos(3,3));
  list.corners.push_back(Pos(4,4));
  EXPECT_EQ(-1, list.next(Region(Pos(3,3),Pos(3,3)), 1));
}
TEST(CornerListTest, IteratesOverRegion) {
  CornerList list;
  for (auto& p: Region(Pos(0,0), Pos(3,3)))
    list.corners.push_back(p);
  Region r(Pos(1,1), Pos(2,2));

  int count = 0;
  for (int i = list.find(r); i >= 0; i = list.next(r, i)) {
    ++count;
    const auto& p = list.corners[i];
    EXPECT_LT(0, p.x) << p.y;
    EXPECT_LT(0, p.y);
    EXPECT_GT(3, p.x) << p.y;
    EXPECT_GT(3, p.y);
  }
  EXPECT_EQ(4, count);
}

TEST(CornerListTest, IteratesOverLargerRegion) {
  CornerList list;
  for (auto& p: Region(Pos(0,0), Pos(11,23)))
    list.corners.push_back(p);
  Region r(Pos(7,5), Pos(9,16));

  int count = 0;
  for (int i = list.find(r); i >= 0; i = list.next(r, i)) {
    ++count;
    const auto& p = list.corners[i];
    EXPECT_LE(r.ll.x, p.x) << p.y;
    EXPECT_LE(r.ll.y, p.y);
    EXPECT_GE(r.ur.x, p.x) << p.y;
    EXPECT_GE(r.ur.y, p.y);
  }
  EXPECT_EQ((r.ur.x - r.ll.x + 1) * (r.ur.y - r.ll.y + 1), count);
}

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

  for (size_t i = 0; i < result.size(); ++i) {
    for (size_t j = i + 1; j < result.size(); ++j) {
      int dx = abs(result[i].x - result[j].x);
      int dy = abs(result[i].y - result[j].y);
      EXPECT_TRUE(dx > 5 || dy > 5) << "i:" << i << " j:" << j << " i.x:" << result[i].x << " j.x:" << result[j].x;
    }
  }
}

TEST_F(CornersTest, NonMax_AscendingDenseSupression) {
  vector<faster::Corner> corners(2);

  for (size_t i = 0 ; i <= 20; ++i) {
    for (size_t j = 0 ; j <= 20; ++j) {
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

int FindBestCorner(const Octave& img1, const Octave& img2, const CornerList& corners, const Pos& c, bool is_left) {
  Region r;
  if (is_left) {
    r = Region(Pos(0, c.y - 2), Pos(c.x + 3, c.y+2));
  } else {
    r = Region(Pos(c.x - 3, c.y - 2), Pos(1<<28, c.y + 2));
  }

  Patch patch(img1.GetPatch(img1.space().convert(c)));
  int best_score = 1<<30;
  int best_idx = -1;
  for (int idx = corners.find(r); idx >= 0; idx = corners.next(r, idx)) {
    const Pos& p(corners.corners[idx]);
    int score = img2.Score(patch, p); // int score; fpos = o1.SearchPosition(o1.space().convert(p), patch, 3, &score);
    if (score < best_score) {
      best_score = score;
      best_idx = idx;
    }
  }
  if (best_idx < 0)
    return -1;
  if (best_score > 4000)
    return -1;
  //printf("score %d\n", best_score);
  return best_idx;
}

TEST_F(CornersTest, Image) {
  Mat mat = imread("data/Wood1/view1.png", CV_LOAD_IMAGE_GRAYSCALE);
  Octave o(OctaveFromMat(mat));
  auto corners = FindCorners(o, 20, 15);


  Mat mat1 = imread("data/Wood1/view5.png", CV_LOAD_IMAGE_GRAYSCALE);
  Octave o1(OctaveFromMat(mat1));
  auto corners1 = FindCorners(o1, 18, 15);

  for (auto&c : corners.corners) {
    int idx = FindBestCorner(o, o1, corners1, c, true);
    if (idx < 0)
      continue;
    const Pos& p = corners1.corners[idx];

    // Now search in reverse to check we ended up where we started.
    int rev_idx = FindBestCorner(o1, o, corners, p, false);
    if (rev_idx < 0)
      continue;

    const Pos& c1 = corners.corners[rev_idx];
    if (c1 != c) {
      printf("Failed to reverse. [%d,%d] != [%d,%d]\n", c1.x, c1.y, c.x, c.y);
      continue;
    }

    line(mat, Point(c.x, c.y - 3), Point(c.x, c.y + 3), Scalar(0,0,0), 1, 8);
    line(mat, Point(c.x - 3, c.y), Point(c.x + 3, c.y), Scalar(0,0,0), 1, 8);

    line(mat1, Point(p.x, p.y - 3), Point(p.x, p.y + 3), Scalar(0,0,0), 1, 8);
    line(mat1, Point(p.x - 3, p.y), Point(p.x + 3, p.y), Scalar(0,0,0), 1, 8);
  }

  namedWindow( "Mat1", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Mat2", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Mat1", mat);                   // Show our image inside it.
  imshow( "Mat2", mat1);                   // Show our image inside it.
  waitKey(0);

}

TEST_F(CornersTest, Video) {
#if 0

  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::VideoCapture vid("data/VID_20130101_071510.mp4");

  Mat img, t, out, grey;
  while (vid.read(img)) {
    cv::transpose(img, t);
    cv::resize(t, out, Size(t.cols / 2, t.rows / 2));
    cv::cvtColor(out, grey, CV_RGB2GRAY);

    Octave o(OctaveFromMat(grey));

    auto corners = FindCorners(o, 20, 10);
    for (auto&c : corners.corners) {
      line(grey, Point(c.x, c.y - 3), Point(c.x, c.y + 3), Scalar(0,0,0), 1, 8);
      line(grey, Point(c.x - 3, c.y), Point(c.x + 3, c.y), Scalar(0,0,0), 1, 8);
    }

    imshow( "Display window", grey);                   // Show our image inside it.
    waitKey(0);
  }
#endif
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
