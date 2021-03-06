//
// 2. Relative mapping for points. (i.e bearing + range relative to initial observed frame).
// 3. Triangulate points for initialization after there are known poses.
// 9. Use a tree or graph of keyframes.
// 10. Consider loop closing when adding new points.
// 11. Stop processing new frames when stationary.
// 12. Add motion model.
// 13. Extract patches and display in GL.
// 14. Add motor/servo control.
// 15. Run full slam in background thread.
// 16. Run fully threaded.
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <eigen3/Eigen/Eigen>
#include <thread>
#include <atomic>
#include <iostream>
#include <fstream>
#include <mutex>
#include <chrono>
#include <ctime>

#include "opencv2/opencv.hpp"
#include "localmap.h"
#include "slam.h"
#include "matcher.h"
#include "vehicle.h"
#include "video.h"


DEFINE_bool(drawdebug, true, "Show debugging display");
DEFINE_bool(move, false, "Run test move sequence");
DEFINE_bool(slam, true, "Run slam solver");
DEFINE_string(save, "", "Save images to specified directory");
DEFINE_string(load, "", "Load images from specified directory");

using namespace std;
using namespace cv;
using namespace Eigen;

int debug = 0;

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
  auto p = point; // cam.Distort(point);
  Point2f a(p(0), p(1));
  line(*out, a - Point2f(size,size), a + Point2f(size,size), color, 1, 8);
  line(*out, a - Point2f(size,-size), a + Point2f(size,-size), color, 1, 8);
}

void DrawLine(cv::Mat* out,
    const Camera& cam,
    const Vector2d& from,
    const Vector2d& to,
    Scalar color) {
  Vector2d p = from; // cam.Distort(from);
  Point2f a(p(0), p(1));
  p = to; // cam.Distort(to);
  Point2f b(p(0), p(1));
  line(*out, a, b, color, 1, 8);
}

void DrawText(cv::Mat* out, const Camera& cam, const string& str, const Vector2d& pos) {
  Vector2d p = pos; // cam.Distort(pos);
  Point2f a(p(0), p(1));
  putText(*out, str, a, FONT_HERSHEY_PLAIN, 0.7, Scalar(0,0,255)); 
}
void DrawDebug(const LocalMap& map, const Camera& cam, Mat* img) {
  Mat& out = *img;
  int frame_id = map.frames.back()->id();
  CHECK_EQ(frame_id, map.frames.size() - 1);

  for (const auto& point : map.points) {
    // Always has at least one observation.
    const Observation* obs = point->observation(-1);

    // If it doesn't appear in this frame, but does appear in the previous
    // frame, draw it as a blue cross.
    if (obs->frame->id() == frame_id - 1) {
      DrawCross(&out, cam, obs->pt, 2, Scalar(255,0,0));
      continue;
    }

    // If it doesn't appear in this frame, skip it.
    if (obs->frame->id() != frame_id)
      continue;

    // If it has only 1 observation, it's a new point: Draw it as a
    // green cross.
    if (point->num_observations() == 1) {
      DrawCross(&out, cam, obs->pt, 2, Scalar(0,255,0));
      continue;
    }

    // It's a tracked point. Draw the current location as a red cross,
    // and a line back to the previous observation. If the current observation
    // is disabled, then draw it as a white line (else a black line).
    const Observation* prev = point->observation(-2);
    if (prev->frame->id() == frame_id - 1) {
      Scalar c(0,0,0);
      if (obs->disabled()) {
        // Bad match.
        c = Scalar(255,255,255);
        cout << point->id() << " is a bad point\n";
      }
      DrawLine(&out, cam, prev->pt, obs->pt, c);
    }
    DrawCross(&out, cam, obs->pt, 3, Scalar(0,0,255));

    // Label with the point id.
    char buff[20];
    sprintf(buff, "%d", point->id());
    DrawText(&out, cam, buff, obs->pt);
  }
}

cv::Mat left_image;
cv::Mat right_image;
Camera* left_cam;
LocalMap* lmap;
int point_id = 0;

bool have_image = false;

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
  if (!have_image)
    return;
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
    pt = dpt; // left_cam->Undistort(dpt);

    double min_dist = 1e2;
    for (const auto& point : lmap->points) {
      const Observation* obs = point->observation(-1);
      double n = (obs->pt - pt).norm();
      if (n < 50 && n < min_dist) {
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

  Vector2d loc = p->observation(-1)->pt;

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
  const int scale = 8;
  int num = 1;
  for (const auto& morig : plist) {
    const Observation* obs = p->observation(-num++);
    Vector2d center;
    obs->frame->Project(p->location(), &center);
    center = (center - obs->pt);
    center += Vector2d(0.5 * morig.size().width, 0.5 * morig.size().height);
    center *= scale;

    Mat m = morig.clone();
    resize(m, m, m.size() * scale, 0, 0, INTER_NEAREST);

    line(m,
        Point2f(m.size().width / 2., m.size().height / 2. - 5),
        Point2f(m.size().width / 2., m.size().height / 2. + 5),
        Scalar(0, 192, 0), 1, 8);
    line(m,
        Point2f(m.size().width / 2. - 5, m.size().height / 2.),
        Point2f(m.size().width / 2. + 5, m.size().height / 2.),
        Scalar(0, 192, 0), 1, 8);

    if (center(0) > 0 && center(1) > 0 &&
        center(0) < m.size().width - 3 && center(1) < m.size().height - 3) {
      m(Rect(center(0),center(1), 3, 3)) = Scalar(0,0,255);
    } else {
    }

    if ((cursor.x + m.size().width) > rout.size().width) {
      cursor.x = 0;
      cursor.y += m.size().height + margin;
    }
    if (cursor.y + m.size().height >= rout.size().height)
      break;
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

  cv::putText(rout, buff, Point2f(0, 400), FONT_HERSHEY_PLAIN, .7, Scalar(128,255,0));

  cv::imshow("Right", rout);
}

void SolveCameras(LocalMap* map, Slam* slam) {
  // Print some debugging stats to STDOUT.
  // Re-run slam with increasingly tight rejection of outliers.
  //slam->SolveAllFrames(map, 10, false);
  //slam->SolveAllFrames(map, 5, false);
  //slam->SolveAllFrames(map, 2, false);

  map->cameras[0]->Reset();
  map->cameras[1]->Reset();

 // map->frames[0]->translation() << 0,0,0;
  //map->frames[1]->translation() << 150,0,0;
  // Run total bundle adjustment including camera instrinsics.
  slam->SolveAllFrames(map, 2, true);
  map->Stats();
  printf("k1 k2 k3 fx fy cx cy\n");
  printf(" %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
      map->cameras[0]->k[0],
      map->cameras[0]->k[1],
      map->cameras[0]->k[2],
      map->cameras[0]->k[3],
      map->cameras[0]->k[4],
      map->cameras[0]->k[5],
      map->cameras[0]->k[6]
      );
  printf(" %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
      map->cameras[1]->k[0],
      map->cameras[1]->k[1],
      map->cameras[1]->k[2],
      map->cameras[1]->k[3],
      map->cameras[1]->k[4],
      map->cameras[1]->k[5],
      map->cameras[1]->k[6]
      );
  slam->ReprojectMap(map);

  map->cameras[0]->Reset();
  map->cameras[1]->Reset();
  printf("k1 k2 k3 fx fy cx cy\n");
  printf(" %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
      map->cameras[0]->k[0],
      map->cameras[0]->k[1],
      map->cameras[0]->k[2],
      map->cameras[0]->k[3],
      map->cameras[0]->k[4],
      map->cameras[0]->k[5],
      map->cameras[0]->k[6]
      );
  printf(" %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\n",
      map->cameras[1]->k[0],
      map->cameras[1]->k[1],
      map->cameras[1]->k[2],
      map->cameras[1]->k[3],
      map->cameras[1]->k[4],
      map->cameras[1]->k[5],
      map->cameras[1]->k[6]
      );
  cv::waitKey(0);
  slam->SolveAllFrames(map, 2, false);
}

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

std::atomic<bool> is_moving(false);

void TestMove() {
  Vehicle v;

  sleep(1);

  double turn = 0.75;
  double speed = 0.18;
  for (int i = 0; i < 8; ++i) {
    v.Turn(turn);
    v.Speed(-speed);
    sleep(2);
    v.Speed(0);

    v.Speed(speed);
    v.Turn(-turn);
    sleep(2);
    v.Speed(0);
  }
  v.Turn(0);
  v.Speed(0);
  is_moving.store(false);
}

std::mutex mu;
std::vector<pair<int, Mat*>> fbuffer;
void WriteImages() {
  while (is_moving.load()) {
    pair<int, Mat*> pair{0, NULL};

    mu.lock();
    if (fbuffer.size()) {
      pair = fbuffer.back();
      fbuffer.pop_back();
    }
    mu.unlock();

    if (!pair.second) {
      std::chrono::milliseconds dura(50);
      std::this_thread::sleep_for(dura);
      continue;
    }

    char b[100];
    sprintf(b, "%08d.png", pair.first);

    cv::imwrite(FLAGS_save + "/" + b, *(pair.second));

    fprintf(stderr, "## Wrote frame %d\n", pair.first);
    delete pair.second;
  }
}

class ScopedTimer {
 public:
  ScopedTimer(const string& name) : name_(name) {
    start_ = std::chrono::system_clock::now();
  }
  ~ScopedTimer() {
    end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_ - start_;

    std::cout << "TIMER: ";
    std::cout << name_;
    std::cout << ": ";
    std::cout << elapsed_seconds.count() << "\n";
  }

 private:
  string name_;
  std::chrono::time_point<std::chrono::system_clock> start_;
  std::chrono::time_point<std::chrono::system_clock> end_;
};

int main(int argc, char*argv[]) {
  // gl_init(argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_drawdebug) {
    cv::namedWindow("Left", CV_WINDOW_AUTOSIZE );// Create a window for display.
    cv::namedWindow("Right", CV_WINDOW_AUTOSIZE );// Create a window for display.
    cv::moveWindow("Left", 1440, 0);
    cv::moveWindow("Right", 1440, 780);

    setMouseCallback("Left", CallBackFunc, NULL);
  }

  vector<std::thread*> threads;
  is_moving.store(true);
  if (FLAGS_move) {
    threads.push_back(new std::thread(TestMove));
  }
  if (!FLAGS_save.empty()) {
    threads.push_back(new std::thread(WriteImages));
    threads.push_back(new std::thread(WriteImages));
    threads.push_back(new std::thread(WriteImages));
  }

  std::unique_ptr<ImageSource> cam;
  if (!FLAGS_load.empty()) {
    cam.reset(new ImageSourceFiles(FLAGS_load));
  } else if (argc == 1) {
    cam.reset(
        new ImageSourceDuo(
          new VideoDev("/dev/video0", 4),
          new VideoDev("/dev/video1", 4)));
  } else if (argc == 2) {
    cam.reset(new ImageSourceMono(argv[1]));
  } else if (argc == 3) {
    cam.reset(
        new ImageSourceDuo(
            new ImageSourceMono(argv[1]),
            new ImageSourceMono(argv[2])));
  } else {
    printf("Too many args\n");
    return 1;
  }
  cam->Init();
 
  // Our knowledge of the 3D world. 
  LocalMap map;
  lmap = &map;

  // We assume two cameras, with frames from 'cam'
  // alternating between them. Initialize with the
  // distortion parameters. k1, k2, k3, fx, fy, cx, cy.
  double focal = 416;
  map.AddCamera(new Camera{
      //-0.11148,  0.18131, -0.00085, 512.96132, -515.11507, 314.17485, 241.31441
      0, 0, 0, focal, -focal, 320, 240
      });
  map.AddCamera(new Camera{
      //-0.12310,  0.18615,  0.01386, 513.92203, -516.38275, 293.13978, 230.27529
      0, 0, 0, focal, -focal, 320, 240
      });

  // Initialize the cameras.
  map.cameras[0]->Reset();
  map.cameras[1]->Reset();

  // A feature tracker. Holds internal state of the previous
  // images.
  Matcher tracking;

  // Bundle adjustment.
  Slam slam;

  // Assumed distance between the two cameras.
  const double kBaseline = 150.;

  // The previous image; used for displaying debugging information.
  Mat prev;

  // Which camera the previous frame came from.
  int camera = 0;
  while (is_moving.load()) {
    ScopedTimer timer_main("Main loop");
    have_image = false;
    camera ^= 1;

    Frame* frame_ptr = map.AddFrame(map.cameras[camera].get());
    int frame_id = frame_ptr->id();
    printf("\n============== Frame %d\n", frame_id);

    //debug = (frame_id > 9);
    //debug = true;
    // Fetch the next image.
    Mat color;
    {
      ScopedTimer timer_main("camera");
      if (!cam->GetObservation(camera, frame_id, &color))
        break;
    }

    if (!FLAGS_save.empty()) {
      Mat* m = new Mat;
      *m = color.clone();
      int count = 0;
      mu.lock();
      fbuffer.push_back({frame_id, m});
      count = fbuffer.size();
      mu.unlock();

      printf("# Fbuffer is %d images\n", count);
    }

    // Blur it slightly: stddev=1
    //GaussianBlur(color, color, Size(5, 5), 1, 1);

    // Add a new frame to the LocalMap (and with it, a new pose).
    // Compute the an initial pose estimate. We assume two cameras separated
    // by 150mm along the X axis.
    if (map.frames.size() == 1) {
      frame_ptr->translation() = Vector3d::Zero();
      frame_ptr->rotation().setIdentity();
    } else if (map.frames.size() == 2) {
      frame_ptr->translation() = Vector3d::UnitX() * kBaseline;
 //     frame_ptr->translation()+= -Vector3d::UnitZ() * kBaseline;
      frame_ptr->rotation() = map.frames[frame_id - 1]->rotation();
    } else {
      // Initialize pose from the two frames ago. (i.e. the previous frame
      // for this camera).
      frame_ptr->translation() = map.frames[frame_id - 2]->translation();
      frame_ptr->rotation() = map.frames[frame_id - 2]->rotation();
    }

    // Slam error threshold.
    const double kErrorThreshold = 5.;

    // Track features against the new image, and fill them into
    // the LocalMap.
    if (FLAGS_slam) {
      tracking.Track(color, frame_ptr, camera, &map, 
          [&]() -> bool {
            return slam.SolveFramePose(frame_ptr->previous(), frame_ptr);
          });
      frame_ptr->Commit();
    }


    // If there's not a previous image, then we can't run comparisons
    // against it: Just skip to getting another image.
    if (prev.size() != color.size()) {
      prev = color;
      continue;
    }

    if (FLAGS_slam) {
      // Run bundle adjustment, first against all the new frame pose
      // (and all world points) while holding all other frame poses
      // constant.
     
      if (slam.SolveFrames(&map,
              2, 5,  // Present 5 frames, solve 2.
              2.)) {
        slam.ReprojectMap(&map);
        map.Clean(kErrorThreshold);
      }
      // Occasionally run bundle adjustment over the previous 10 frames.
      if (frame_id < 10 || (frame_id % 5) == 0) {
        // Then solve all frame poses.
        //slam.Run(&map, nullptr);
        // Solve the last 10 frame poses.
        if (!slam.SolveFrames(&map,
              10, 20,  // Solve 10 frames out of 20 presented.
              2.))
          break;
        slam.ReprojectMap(&map);
        map.Clean(kErrorThreshold);
      }

      map.ApplyEpipolarConstraint();

      // Rotate and scale the map back to a standard baseline.
      double err1 = slam.ReprojectMap(&map);
      map.Normalize();
      double err2 = slam.ReprojectMap(&map);
      CHECK_NEAR(err1, err2, 1e-1);
    }

    // Draw observation history onto the left frame.
    if (FLAGS_drawdebug) {
      Mat out1 = prev.clone();
      char buff[100];
      sprintf(buff, "frame %3d camera %3d", frame_ptr->id() - 1, camera ^ 1);
      cv::putText(out1, buff, Point2f(100, 300), FONT_HERSHEY_PLAIN, .7, Scalar(128,255,0));
      DrawDebug(map, *(map.cameras[camera ^ 1].get()), &out1);

      Mat out2 = color.clone();
      sprintf(buff, "frame %3d camera %3d", frame_ptr->id(), camera);
      cv::putText(out2, buff, Point2f(100, 300), FONT_HERSHEY_PLAIN, .7, Scalar(128,255,0));
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
    }
    prev = color;
    have_image = true;

    //if (!(frame_id% 20))
    if (FLAGS_drawdebug)
      cv::waitKey(0);

    if (0 && (frame_id % 20) == 0) {
      SolveCameras(&map, &slam);
    }

    if (frame_id == 400 && FLAGS_drawdebug) break;
  }

  //map.Stats();
  //slam.Run(&map, 1, nullptr);

  FILE* f = fopen("/tmp/z", "w");
  DumpMap(&map, f);
  fclose(f);

  printf("Iterations: %d, error %f\n",
         slam.iterations(),
         slam.error());

  for (const auto& t : threads) {
    t->join();
    delete t;
  }
  return 0;

}
