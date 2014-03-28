//
// 1. Normalize slightly changes if points are unset or not. why?
// 2. Relative mapping for points. (i.e bearing + range relative to initial observed frame).
// 3. Triangle points for initialization after there are known poses.
//
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

#include "localmap.h"
#include "slam.h"
#include "matcher.h"
#include "corners.h"


using namespace std;
using namespace cv;
using namespace Eigen;

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
void DrawCross(cv::Mat* out, const Camera& cam, const Vector2d& point, int size, Scalar color) {
  auto p = cam.Distort(point);
  Point2f a(p(0), p(1));
  line(*out, a - Point2f(size,size), a + Point2f(size,size), color, 1, 8);
  line(*out, a - Point2f(size,-size), a + Point2f(size,-size), color, 1, 8);
}

void DrawLine(cv::Mat* out,
    const Camera& cam,
    const Vector2d& from,
    const Vector2d& to,
    Scalar color) {
  Vector2d p = cam.Distort(from);
  Point2f a(p(0), p(1));
  p = cam.Distort(to);
  Point2f b(p(0), p(1));
  line(*out, a, b, color, 1, 8);
}

void DrawText(cv::Mat* out, const Camera& cam, const string& str, const Vector2d& pos) {
  Vector2d p = cam.Distort(pos);
  Point2f a(p(0), p(1));
  putText(*out, str, a, FONT_HERSHEY_PLAIN, 0.7, Scalar(0,0,255)); 
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

// Interface: A source of images. Typically a camera or
// a video file.
class ImageSource {
 protected:
  ImageSource() {}
 public:
  virtual ~ImageSource() {}
  virtual bool GetObservation(int camera, Mat* img) = 0;
  virtual bool Init() = 0;
};

// Source images from a single video file.
class ImageSourceMono : public ImageSource {
 public:
  ImageSourceMono(const char* filename) : filename_(filename), cam_(filename) {}
  bool Init() {
    if (!cam_.isOpened()) {
      printf("Failed to open video file: %s\n", filename_);
      return false;
    }
    return true;
  }
  virtual bool GetObservation(int, Mat* img) {
    return cam_.read(*img);
  }

 private:
  const char* filename_;
  VideoCapture cam_;
};

// Source images from two video files.
class ImageSourceDuo : public ImageSource {
 public:
  ImageSourceDuo(const char* filename1, const char* filename2) :
      filename1_(filename1), filename2_(filename2),
      cam1_(filename1), cam2_(filename2) {}

  bool Init() {
    if (!cam1_.isOpened()) {
      printf("Failed to open video file: %s\n", filename1_);
      return false;
    }
    if (!cam2_.isOpened()) {
      printf("Failed to open video file: %s\n", filename2_);
      return false;
    }
    return true;
  }

  virtual bool GetObservation(int camera, Mat* img) {
    if (camera == 0)
      return cam1_.read(*img);
    else
      return cam2_.read(*img);
  }

 private:
  const char* filename1_;
  const char* filename2_;
  VideoCapture cam1_;
  VideoCapture cam2_;
};


void DrawDebug(const LocalMap& map, const Camera& cam, Mat* img) {
  Mat& out = *img;
  int frame_num = map.frames.size() - 1;

  for (const auto& point : map.points) {
    if (point->last_frame() == frame_num - 1) {
      DrawCross(&out, cam, point->last_point(), 2, Scalar(255,0,0));
      continue;
    }
    if (abs(point->last_frame()) != frame_num)
      continue;

    if (point->num_observations() == 1) {
      DrawCross(&out, cam, point->last_point(), 2, Scalar(0,255,0));
      continue;
    }

    int num = point->num_observations();
    if (point->observations_[num - 2].frame_idx == frame_num - 1) {
      Scalar c(0,0,0);
      if (point->observations_[num - 1].frame_idx == -frame_num) {
        // Bad match.
        c = Scalar(255,255,255);
        cout << point->id_ << " is a bad point\n";
      }
      DrawLine(&out, cam, point->observations_[num - 2].pt,
          point->observations_[num - 1].pt, c);
    }
    DrawCross(&out, cam, point->last_point(), 4, Scalar(0,0,255));

    char buff[20];
    sprintf(buff, "%d", point->id_);
    DrawText(&out, cam, buff, point->last_point());
  }
}

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

  std::unique_ptr<ImageSource> cam;
  if (argc == 2) {
    cam.reset(new ImageSourceMono(argv[1]));
  } else if (argc == 3) {
    cam.reset(new ImageSourceDuo(argv[1], argv[2]));
  }
 
  // Our knowledge of the 3D world. 
  LocalMap map;

  // We assume two cameras, with frames from 'cam'
  // alternating between them.

  // [CAMERA_PARAMS] Left
  // resolution=[640 480]
  // cx=287.03691
  // cy=228.17558
  // fx=530.05895
  // fy=530.23204
  // dist=[-1.442760e-01 3.246676e-01 1.588004e-04 -1.096403e-03 0.000000e+00]
  //
  // [CAMERA_PARAMS] Right
  // resolution=[640 480]
  // cx=312.14364
  // cy=233.92664
  // fx=525.75698
  // fy=526.16432
  // dist=[-1.091669e-01 2.201787e-01 -1.866669e-03 1.632135e-04 0.000000e+00]
  //
  Camera* cam_model = map.AddCamera();
  cam_model->center << 287, 228;
  cam_model->focal << 530.1, 530.2;
  cam_model->k1 = -1.442760e-01;
  cam_model->k2 = 3.246676e-01;
  cam_model->p1 = 1.588004e-04;
  cam_model->p2 = -1.096403e-03;
  cam_model->k3 = 0.000000e+00;

  cam_model = map.AddCamera();
  cam_model->center << 312.14, 233.93;
  cam_model->focal << 525.76, 526.16;
  cam_model->k1 = -1.091669e-01;
  cam_model->k2 = 2.201787e-01;
  cam_model->p1 = -1.866669e-03;
  cam_model->p2 = 1.632135e-04;
  cam_model->k3 = 0.000000e+00;

  // A feature tracker. Holds internal state of the previous
  // images.
  Matcher tracking;

  // Bundle adjustment.
  Slam slam;

  // The previous image; used for displaying debugging information.
  Mat prev;
  while (1) {
    int frame_num = map.frames.size();
    int camera = frame_num & 1;

    // Fetch the next image.
    Mat color, grey;
    if (!cam->GetObservation(camera, &color))
      break;

    // Blur it slightly: stddev=1
    GaussianBlur(color, color, Size(5, 5), 1, 1);

    // Make a grey-scale version of it.
    cvtColor(color, grey, CV_RGB2GRAY);

    // Add a new frame to the LocalMap (and with it, a new pose).
    Frame* frame_ptr = map.AddFrame(map.cameras[camera].get());
    // Compute the an initial pose estimate. We assume two cameras separated
    // by 150mm along the X axis.
    frame_ptr->pose.translation_ -= (camera * 2. - 1.) * Vector3d::UnitX() * 150.;

    // Track features against the new image, and fill them into
    // the LocalMap.
    tracking.Track(grey, frame_ptr, &map);

    // If there's not a previous image, then we can't run comparisons
    // against it: Just skip to getting another image.
    if (prev.size() != color.size()) {
      prev = color;
      continue;
    }

    //UpdateMap(&map, frame, normed_points, descriptors);

    // Run bundle adjustment, first against all the new frame pose
    // (and all world points) while holding all other frame poses
    // constant.
    const double kErrorThreshold = 5.;
    do {
      // Just solve the current frame while holding all others
      // constant.
      slam.Run(&map,
          [=](int frame_idx) ->bool{ return frame_idx == frame_num; }
          );
      slam.ReprojectMap(&map);
    } while (!map.Clean(kErrorThreshold));

    // Then solve all frame poses.
    slam.Run(&map, nullptr);
    map.Clean(kErrorThreshold);

    // Rotate and scale the map back to a standard baseline.
    double err1 = slam.ReprojectMap(&map);
    map.Normalize();
    double err2 = slam.ReprojectMap(&map);

    CHECK_NEAR(err1, err2, 1e-10);


    // Print some debugging stats to STDOUT.
    map.Stats();


    Mat blend;
    addWeighted(prev, 0.5, color, 0.5, 0, blend);

    // Draw observation history onto the left frame.
    Mat out1 = prev.clone();
    DrawDebug(map, *(map.cameras[camera ^ 1].get()), &out1);
    Mat out2 = color.clone();
    DrawDebug(map, *(map.cameras[camera].get()), &out2);

    cv::imshow("Left", out1);
    cv::imshow("Right", out2);
    prev = color;
    cv::waitKey(0);
    if (frame_num >= 100)
      break;
    //cv::waitKey(0);
  }

  DumpMap(&map);

  printf("Iterations: %d, error %f\n",
         slam.iterations(),
         slam.error());
  return 0;
}
