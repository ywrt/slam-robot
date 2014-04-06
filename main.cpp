//
// 2. Relative mapping for points. (i.e bearing + range relative to initial observed frame).
// 3. Triangulate points for initialization after there are known poses.
// 4. First solve frame pose, and then unknown points.
// 6. Bundle adjustment failure.
// 7. Remove points behind cameras.
// 8. Don't solve the entire world every time!
// 9. Use a tree or graph of keyframes.
// 10. Consider loop closing when adding new points.
// 11. Stop processing new frames when stationary.
// 12. Add motion model.
// 13. Extract patches and display in GL.
//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "opencv2/opencv.hpp"

#include <eigen3/Eigen/Eigen>

#include <glog/logging.h>

#include "localmap.h"
#include "slam.h"
#include "matcher.h"
#include "gl.h"

using namespace std;
using namespace cv;
using namespace Eigen;

void DumpMap(LocalMap* map, FILE* out) {
  for (auto& f : map->frames) {
    if ((f->id() & 1) != 0) continue;
    Vector3d pos = f->position();
    //const auto& rot = f->rotation().coeffs();
    fprintf(out, "%f  %f  %f\n",
           pos[0], pos[1], pos[2]);
  }
  fprintf(out, "\n");
  for (auto& f : map->frames) {
    if ((f->id() & 1) != 1) continue;
    Vector3d pos = f->position();
    fprintf(out, "%f  %f  %f\n",
           pos[0], pos[1], pos[2]);
  }
  fprintf(out, "\n");

  for (auto& p : map->points) {
    if (!p->slam_usable())
      continue;
    Vector3d pos = p->position();
    if (pos.norm() > 4000)
      continue;
    fprintf(out, "%f %f %f\n\n", pos(0), pos(1), pos(2));
  }

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
  int frame_id = map.frames.back()->id();
  CHECK_EQ(frame_id, map.frames.size() - 1);

  for (const auto& point : map.points) {
    // Always has at least one observation.
    const auto& obs = point->observation(-1);

    // If it doesn't appear in this frame, but does appear in the previous
    // frame, draw it as a blue cross.
    if (obs.frame->id() == frame_id - 1) {
      DrawCross(&out, cam, obs.pt, 2, Scalar(255,0,0));
      continue;
    }

    // If it doesn't appear in this frame, skip it.
    if (obs.frame->id() != frame_id)
      continue;

    // If it has only 1 observation, it's a new point: Draw it as a
    // green cross.
    if (point->num_observations() == 1) {
      DrawCross(&out, cam, obs.pt, 2, Scalar(0,255,0));
      continue;
    }

    // It's a tracked point. Draw the current location as a red cross,
    // and a line back to the previous observation. If the current observation
    // is disabled, then draw it as a white line (else a black line).
    const auto& prev = point->observation(-2);
    if (prev.frame->id() == frame_id - 1) {
      Scalar c(0,0,0);
      if (obs.disabled()) {
        // Bad match.
        c = Scalar(255,255,255);
        cout << point->id() << " is a bad point\n";
      }
      DrawLine(&out, cam, prev.pt, obs.pt, c);
    }
    DrawCross(&out, cam, obs.pt, 3, Scalar(0,0,255));

    // Label with the point id.
    char buff[20];
    sprintf(buff, "%d", point->id());
    DrawText(&out, cam, buff, obs.pt);
  }
}

cv::Mat left_image;
cv::Mat right_image;
Camera* left_cam;
LocalMap* lmap;
int point_id = 0;

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
  TrackedPoint* p = nullptr;

  if  ( event == EVENT_LBUTTONDOWN ) {
    point_id++;
    for (const auto& point : lmap->points) {
      if (point->id() != point_id)
        continue;
      p = point.get();
      break;
    }
  } else if  ( event == EVENT_RBUTTONDOWN ) {
    cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    cv::Point2f a(x, y);

    Vector2d dpt, pt;
    dpt << x, y;
    pt = left_cam->Undistort(dpt);

    double min_dist = 1e2;
    for (const auto& point : lmap->points) {
      const auto& obs = point->observation(-1);
      double n = (obs.pt - pt).norm();
      if (n < 1e-1 && n < min_dist) {
        p = point.get();
        min_dist = n;
      }
    }
  }

  char buff[1000] = "";
  if (!p)
    return;

  cv::Mat out = left_image.clone();

  point_id = p->id();
  sprintf(buff, "%d", p->id());

  Vector2d loc = left_cam->Distort(p->observation(-1).pt);

  cv::putText(out, buff, Point2f(loc(0), loc(1)), FONT_HERSHEY_PLAIN, 0.7, Scalar(128,255,0));
  cv::imshow("Left", out);

  const auto& patches = GetPatches();
  const auto& iter = patches.find(p->id());
  if (iter == patches.end())
    return;
  const auto& plist = iter->second;

  Mat rout(480, 640, CV_8UC3); //right_image.type());
  rout = Scalar(0,0,0);

  Point2f cursor(0,0);
  const int margin = 2;
  const int scale = 5;
  int num = 1;
  for (const auto& morig : plist) {
    const auto& obs = p->observation(-num++);
    Vector2d center;
    obs.frame->Project(p->location(), &center);
    center = (left_cam->Distort(center) - left_cam->Distort(obs.pt));
    center += Vector2d(7.5, 7.5);
    center *= scale;

    Mat m;
    cvtColor(morig.clone(), m, CV_GRAY2RGB);
    resize(m, m, m.size() * scale, 0, 0, INTER_NEAREST);

    if (center(0) > 0 && center(1) > 0 &&
        center(0) < m.size().width - 3 && center(1) < m.size().height - 3) {
      m(Rect(center(0),center(1), 3, 3)) = Scalar(0,0,255);
    } else {
    }

    if ((cursor.x + m.size().width) > rout.size().width) {
      cursor.x = 0;
      cursor.y += m.size().height + margin;
    }
    m.copyTo(rout(Rect(cursor.x, cursor.y, m.size().width, m.size().height)));
    cursor.x += m.size().width + margin;
  }

  sprintf(buff, "p %3d: dist %6.1f [%9.1f, %9.1f, %9.1f]",
      p->id(),
      p->position().norm(),
      p->position()[0],
      p->position()[1],
      p->position()[2]
      );

  cv::putText(rout, buff, Point2f(0, 200), FONT_HERSHEY_PLAIN, .7, Scalar(128,255,0));

  cv::imshow("Right", rout);
}


int main(int argc, char*argv[]) {
  // gl_init(argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (argc < 2) {
    fprintf(stderr, "Usage: slam left.avi right.avi\n");
    exit(1);
  }

  cv::namedWindow("Left", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::namedWindow("Right", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::moveWindow("Left", 1440, 0);
  cv::moveWindow("Right", 1440, 780);

  setMouseCallback("Left", CallBackFunc, NULL);

  std::unique_ptr<ImageSource> cam;
  if (argc == 2) {
    cam.reset(new ImageSourceMono(argv[1]));
  } else if (argc == 3) {
    cam.reset(new ImageSourceDuo(argv[1], argv[2]));
  }
 
  // Our knowledge of the 3D world. 
  LocalMap map;
  lmap = &map;

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
  map.AddCamera();
  map.AddCamera();
  Camera* cam_model = map.cameras[1].get();
  cam_model->center << 320, 240; // 287, 228;
  cam_model->center << 287, 228;
  cam_model->focal << 530.1, 530.2;
  cam_model->k1 = -5e-2; // -1.442760e-01;
  //cam_model->k2 = 3.246676e-01;
  //cam_model->p1 = 1.588004e-04;
  //cam_model->p2 = -1.096403e-03;
  //cam_model->k3 = 0.000000e+00;

  cam_model = map.cameras[0].get();
  cam_model->center << 320, 240; // 312.14, 233.93;
  cam_model->center << 312.14, 233.93;
  cam_model->focal << 525.76, 526.16;
  cam_model->k1 = -4e-2;
  //cam_model->k1 = -1.091669e-01;
  //cam_model->k2 = 2.201787e-01;
  //cam_model->p1 = -1.866669e-03;
  //cam_model->p2 = 1.632135e-04;
  //cam_model->k3 = 0.000000e+00;

  // A feature tracker. Holds internal state of the previous
  // images.
  Matcher tracking;

  // Bundle adjustment.
  Slam slam;

  const double kBaseline = 150.;
  // The previous image; used for displaying debugging information.
  Mat prev;
  int camera = 1;
  while (1) {
    camera ^= 1;

    Frame* frame_ptr = map.AddFrame(map.cameras[camera].get());
    frame_ptr->dist = kBaseline;
    int frame_id = frame_ptr->id();
    printf("\n============== Frame %d\n", frame_id);

    // Fetch the next image.
    Mat color, grey;
    if (!cam->GetObservation(camera, &color))
      break;

    // Blur it slightly: stddev=1
    //GaussianBlur(color, color, Size(5, 5), 1, 1);

    // Make a grey-scale version of it.
    cvtColor(color, grey, CV_RGB2GRAY);

    // Add a new frame to the LocalMap (and with it, a new pose).
    // Compute the an initial pose estimate. We assume two cameras separated
    // by 150mm along the X axis.
    if (map.frames.size() == 1) {
      frame_ptr->translation() = Vector3d::Zero();
    } else if (map.frames.size() == 2) {
      frame_ptr->translation() = -Vector3d::UnitX() * kBaseline;
    } else {
      // Initialize pose from the two frames ago. (i.e. the previous frame
      // for this camera).
      frame_ptr->translation() = map.frames[frame_id - 2]->translation();
      frame_ptr->rotation() = map.frames[frame_id - 2]->rotation();
    }

    for (unsigned int i = 2 ; i < map.frames.size() - 1; ++i) {
      double d = (map.frames[i-2]->position() - map.frames[i]->position()).norm();
      map.frames[i]->dist = sqrt(d*d/4 + kBaseline * kBaseline);
    }

    // Track features against the new image, and fill them into
    // the LocalMap.
    tracking.Track(grey, frame_ptr, &map);

    // If there's not a previous image, then we can't run comparisons
    // against it: Just skip to getting another image.
    if (prev.size() != color.size()) {
      prev = color;
      continue;
    }

    // Run bundle adjustment, first against all the new frame pose
    // (and all world points) while holding all other frame poses
    // constant.
    const double kErrorThreshold = 5.;
    do {
      // Just solve the current frame pose while holding all other frame
      // poses constant.
      if (!slam.Run(&map,
            4 * kErrorThreshold * 1e-3,
            [=](Frame* frame) -> bool {
              return frame->id() == frame_id;
            })) {
        break;  // Failed to run SLAM.
      }
      slam.ReprojectMap(&map);
    } while (!map.Clean(kErrorThreshold * 4));

    if (frame_id < 10 || (frame_id % 5) == 0) {
      // Then solve all frame poses.
      //slam.Run(&map, nullptr);
      // Solve the last 10 frame poses.
      slam.Run(&map,
          kErrorThreshold * 1e-5,
          [=](Frame* frame)-> bool {
            return frame->id() >= (frame_id - 10);
          });
      map.Clean(kErrorThreshold);
    }

    // Rotate and scale the map back to a standard baseline.
    double err1 = slam.ReprojectMap(&map);
    map.Normalize();
    double err2 = slam.ReprojectMap(&map);

    CHECK_NEAR(err1, err2, 1e-10);



    Mat blend;
    addWeighted(prev, 0.5, color, 0.5, 0, blend);

    // Draw observation history onto the left frame.
    Mat out1 = prev.clone();
    DrawDebug(map, *(map.cameras[camera ^ 1].get()), &out1);
    Mat out2 = color.clone();
    DrawDebug(map, *(map.cameras[camera].get()), &out2);

    if (camera&1) {
      cv::imshow("Left", out1);
      cv::imshow("Right", out2);
      left_image = out1;
      left_cam = map.cameras[0].get();
    } else {
      cv::imshow("Left", out2);
      cv::imshow("Right", out1);
      left_image = out2;
      left_cam = map.cameras[0].get();
    }
    prev = color;
    //if (frame_num >= 200)
    //  cv::waitKey(0);
    if ((frame_id % 5) == 0) {
      // Print some debugging stats to STDOUT.
 //     map.Stats();
      cv::waitKey(50);
    }

    if (frame_id == 400) break;
  }

  map.Stats();
  slam.Run(&map, 1e-5, nullptr);

  FILE* f = fopen("/tmp/z", "w");
  DumpMap(&map, f);
  fclose(f);

  printf("Iterations: %d, error %f\n",
         slam.iterations(),
         slam.error());
  return 0;

}
