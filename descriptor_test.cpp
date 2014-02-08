#include "opencv2/opencv.hpp"
#include "octave.h"
#include <stdio.h>

#include "descriptor.h"

#include <gtest/gtest.h>

namespace {

using namespace cv;

// The fixture for testing class Descriptor.
class DescriptorTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  DescriptorTest() : width_(0), height_(0) {
    // You can do set-up work for each test here.
  }

  virtual ~DescriptorTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
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
    o_.Smooth();

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

TEST_F(DescriptorTest, Range) {
  Descriptor *l = new Descriptor[width_ * height_];
  int count = 0;
  for (int y = 15; y < height_ - 16; y ++) {
    for (int x = 15; x < width_ - 16; x ++) {
      int score = o_.ScoreCorner(Pos(x, y));
      if (score < 2000) continue;
      //std::cout << score << " score\n";
      l[count++] = Descriptor(o_, Pos(x, y));
      int s = l[count-1].score();
      if (s < 70)
        count--;
     // else
     //   std::cout << s << "\n";
    }
  }
  //std::cout << count << " count\n";
  
  int dist[300]={0,};
  for (int i = 0; i < count; ++i) {
    int min = 299;
    for (int j = i + 1 ; j < count; ++j) {
      int d = l[i].distance(l[j]);
      if (d < min) min = d;
      if (d < 2) {
        printf("Same %2d == %2d : %3d v %3d\n", i, j, l[i].score(), l[j].score());
      }
    }
    //std::cout << min << "\n";
    dist[min]++;
  }
  //std::cout << "\n";
  for (int i = 0; i < 300; ++i) {
    if (!dist[i]) continue;
    //std::cout << i << " " << dist[i] << "\n";
  }
  delete[] l;
}

TEST_F(DescriptorTest, Compare) {
  Descriptor *l = new Descriptor[width_ * height_];
  Pos *p = new Pos[width_ * height_];

  int count = 0;
  for (int y = 15; y < height_ - 16; y ++) {
    for (int x = 15; x < width_ - 16; x ++) {
      int score = o_.ScoreCorner(Pos(x, y));
      if (score < 1000) continue;
      p[count] = Pos(x, y);
      l[count++] = Descriptor(o_, Pos(x, y));

      int s = l[count-1].score();
      if (s < 70)
        count--;
    }
  }

  int good[300] = {0,};
  int bad[300] = {0,};
  for (int y = 15; y < height_ - 16; y ++) {
    for (int x = 15; x < width_ - 16; x ++) {
      int score = o1_.ScoreCorner(Pos(x, y));
      if (score < 1000) continue;
      Descriptor d(o1_, Pos(x, y));
      if (d.score() < 70)
        continue;

      int min = 1000;
      int index;
      for (int i = 0; i < count; ++i) {
        int dist = l[i].distance(d);
        if (dist < min) { index = i; min = dist; }
      }
      //printf("(%3d,%3d) => (%3d, %3d) gave [%-3d, %-3d] @ %d\n", x, y, p[index].x, p[index].y, x-p[index].x, y-p[index].y, min);
      if ((y-p[index].y) > 3 || (p[index].y - y) > 3)
        bad[min]++;
      else
        good[min]++;
    }
  }

  for (int i = 0; i < 300; ++i) {
    if (!good[i] && !bad[i]) continue;
    //printf("%3d: %4d good %4d bad\n", i, good[i], bad[i]);
  }

  delete[] l;
  delete[] p;
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
