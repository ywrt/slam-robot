#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/ordered_groups.h>

#include <vector>
#include <map>

#include <eigen3/Eigen/Eigen>

#include <glog/logging.h>

#include "imgtypes.h"
#include "octaveset.h"
#include "grid.h"

#include "localmap.h"
#include "slam.h"
#include "matcher.h"
#include "corners.h"


using namespace std;
using namespace cv;
using namespace Eigen;


// TODO: Check point visibility before attempting to match.
// TODO: Use descriptor matching to attempt loop closing.
// TODO: Build and use key frames.
// TODO: reduce use of structs.
// TODO: Merge FPos and Vector2d.
// TODO: Better bounds checking for Octave.
// TODO: Better name for FPos (relative pos?)
// TODO: Change FPos to use doubles.
// TODO: Make it real time
// TODO: Use pose estimate for homography estimates.
// TODO: OMPL == Open Motion Planning Library.


void DumpMap(LocalMap* map) {
  for (auto& f : map->frames) {
    printf("(%f,%f,%f,%f) -> (%f, %f, %f)\n",
           f->pose.rotation()[0], f->pose.rotation()[1],
           f->pose.rotation()[2], f->pose.rotation()[3],
           f->pose.translation()[0], f->pose.translation()[1],
           f->pose.translation()[2]);
  }
#if 0
  for (auto& p : map->points) {
    printf("pt %f,%f,%f\n",
           p.data[0], p.data[1], p.data[2]);
  }
#endif

}


// Draw a diagonal cross onto an OpenCV image.
void DrawCross(cv::Mat* out, const Vector2d& point, int size, Scalar color) {
  Point2f a(
      (point(0) + 1)/2 * out->cols,
      (point(1) + 1)/2 * out->rows);
  line(*out, a - Point2f(size,size), a + Point2f(size,size), color, 1, 8);
  line(*out, a - Point2f(size,-size), a + Point2f(size,-size), color, 1, 8);
}

void DrawCross(cv::Mat* out, const Pos& point, int size, Scalar color) {
  Point2f a(point.x, point.y);
  line(*out, a - Point2f(size,size), a + Point2f(size,size), color, 1, 8);
  line(*out, a - Point2f(size,-size), a + Point2f(size,-size), color, 1, 8);
}

void DrawLine(cv::Mat* out,
    const Vector2d& from,
    const Vector2d& to,
    Scalar color) {
  Point2f a(
      (from(0) + 1)/2 * out->cols,
      (from(1) + 1)/2 * out->rows);
  Point2f b(
      (to(0) + 1)/2 * out->cols,
      (to(1) + 1)/2 * out->rows);
  line(*out, a, b, color, 1, 8);
}

// Wrapper for VideoCapture. Can open cameras directly or
// a video file.
class Eye {
 public:
  Eye(int eye) {
    cam_.open(eye);
    cam_.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cam_.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cam_.set(CV_CAP_PROP_FPS, 15);
  }
  Eye(char* filename) {
    cam_.open(filename);
  }

  void grab() { cam_.grab(); }

  void proc() {
    cam_.retrieve(frame_, 0);
    //cv::resize(frame_, frame_, cv::Size(frame_.size().width / 2, frame_.size().height / 2));
    cvtColor(frame_, grey_, CV_RGB2GRAY);
  }

 public:
  VideoCapture cam_;
  Mat frame_, grey_;
};


int main(int argc, char*argv[]) {
  google::InitGoogleLogging(argv[0]);
  if (argc < 2) {
    fprintf(stderr, "Usage: slam left.avi right.avi\n");
    exit(1);
  }

  cv::namedWindow("Left", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::namedWindow("Right", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::moveWindow("Left", 1440, 0);
  cv::moveWindow("Right", 1440, 780);

  Eye left(argv[1]);
  Eye right(argv[2]);

  int frame = -1;

  LocalMap map;
  map.AddCamera();
  map.AddCamera();

  Matcher tracking;

  Slam slam;
  while (1) {
    frame+=2;
    left.grab();
    right.grab();

    left.proc();
    right.proc();

    
    tracking.Track(left.grey_, 0, &map);
    tracking.Track(right.grey_, 1, &map);

    //UpdateMap(&map, frame, normed_points, descriptors);

    Mat blend;
    addWeighted(left.frame_, 0.5, right.frame_, 0.5, 0, blend);

    // Draw observation history ontot left frame.
    Mat& out = blend;
    for (const auto& point : map.points) {
      if (point->last_frame() == frame - 1) {
        DrawCross(&out, point->last_point(), 2, Scalar(255,0,0));
        continue;
      }
      if (point->last_frame() != frame)
        continue;
      if (point->num_observations() == 1) {
        DrawCross(&out, point->last_point(), 2, Scalar(0,255,0));
        continue;
      }
      int num = point->observations_.size();
      for (int i = 0; i < (num - 1); ++i) {
        DrawLine(&out, point->observations_[i].pt,
            point->observations_[i+1].pt, Scalar(0,0,0));
      }
      DrawCross(&out, point->last_point(), 5, Scalar(0,0,255));
    }

    cv::imshow("Left", blend);
    cv::imshow("Right", right.frame_);

    int mod = 1;

    slam.Run(&map, -1, false);
    slam.ReprojectMap(&map);
    map.Clean();

    if (0) {
      slam.Run(&map, -1, false);
      slam.ReprojectMap(&map);
      map.Clean();

      //DumpMap(&map);
    }

    cv::waitKey(0);
    if (frame >= 100)
      break;
    //cv::waitKey(0);
  }
  //slam.Run(&map, -1, true);

  DumpMap(&map);

  printf("Iterations: %d, error %f\n",
         slam.iterations(),
         slam.error());
  return 0;
}
