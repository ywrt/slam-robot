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

void UpdateMap(LocalMap* map,
    int frame,
    const vector<Vector2d>& key_points,
    const Mat& descriptors) {

  CHECK_EQ(key_points.size(), descriptors.rows);

  int hist[512] = {0,};

  // Compute approximate point visibility.
  set<TrackedPoint*> points;
  for (auto& point : map->points) {
    if (point.bad_)
      continue;
    points.insert(&point);  // TODO: actually compute visibility.
  }

  // Match tracked points into the current frame.
  for (int i = 0; i < descriptors.rows;++i) {
    uint8_t* ptr = (uint8_t*) descriptors.ptr(i);
    Descriptor desc(ptr);

    int min_distance = 512;
    TrackedPoint* best_match = NULL;
    for (auto& point : points) {
      if (point->match(ptr, &min_distance))
        best_match = point;
    }
    if (min_distance < 512)
      hist[min_distance/10]++;
    //printf("%3d: dist %d (%d)\n", i, min_distance, best_match);

    if (min_distance < 30) {
      CHECK(best_match != NULL);
      if (best_match->last_frame() == frame) {
        // We've already matched this point in this frame!
        // Mark the point as bad.
        best_match->bad_ = true;
        continue;
      }

      // A useful match
      Observation o;
      o.pt = key_points[i];
      o.frame_ref = frame;
      best_match->observations_.push_back(o);
    } else if (min_distance > 15) {
      // No match and a distinct point.
      // insert this as a new image point.
      map->points.push_back(TrackedPoint());
      TrackedPoint* point = &map->points.back();
      Observation o;
      o.pt = key_points[i];
      o.frame_ref = frame;
      point->observations_.push_back(o);
      point->descriptors_.push_back(desc);
    }
  }

  // Remove tracked points that aren't useful. I.e.
  // tracked points that matched only one frame, and were
  // created more than 3 frames ago.
  // TODO: change map->points to be a vector of pointers
  // and have this erase bad points.
  for (auto& point : map->points) {
    if (point.num_observations() > 1)
      continue;
    if ((frame - point.last_frame()) < 3)
      continue;
    point.bad_ = true;
  }

  for (size_t i = 0; i < 512; ++i) {
    if (!hist[i])
      continue;
    printf("%3zd: %d\n", i, hist[i]);
  }
}

void DumpMap(LocalMap* map) {
  for (auto& f : map->frames) {
    printf("(%f,%f,%f,%f) -> (%f, %f, %f)\n",
           f.rotation()[0], f.rotation()[1],
           f.rotation()[2], f.rotation()[3],
           f.translation()[0], f.translation()[1],
           f.translation()[2]);
  }
#if 0
  for (auto& p : map->points) {
    printf("pt %f,%f,%f\n",
           p.data[0], p.data[1], p.data[2]);
  }
#endif

  printf("focal %f r1 %f r2 %f\n",
         map->camera.data[0],
         map->camera.data[1],
         map->camera.data[2]);
}

struct  Tracking {
  Tracking() :
    curr(new OctaveSet),
    prev {NULL, } {
      for (int i = 0; i < kSearchFrames; ++i) {
        prev[i] = new OctaveSet;
      }
    }

  void ComputeHomography(const Frame& f1, const Frame& f2, Matrix3d* homog) {
    Matrix3d rotate;
    rotate = f2.rotation_ * f1.rotation_.inverse();

    Vector3d translate;
    translate = f2.translation_ - f1.translation_;
    // translate *= focal_length;

    // Now we have camera 1 pointed directly along the Z axis at the
    // origin, and the position(t)+rotation(R) of camera2 w.r.t camera1
    // The homography is thus R + (t*n')/f where n' =[0;0;-1] aka the
    // direction from the projection plane to the camera, and 'f'
    // is the distance from the projection plane to the camera aka
    // the focal length.
    rotate.block<3,1>(0,2) -= translate;

    *homog = AngleAxisd(-M_PI/2, Vector3d::UnitZ()) *
        rotate *
        AngleAxisd(M_PI/2, Vector3d::UnitZ());
  }

  Vector2d ComputePoint(
      const Vector2d& point,
      const Matrix3d& homog) {
    Vector3d v;
    v.block<2,1>(0,0) = point;
    v(2) = 1;

    v = homog * v;
    v.block<2,1>(0,0) /= v(2);
    return v.block<2,1>(0,0);
  }

  int UpdateCorners(LocalMap* map, int frame_num) {
    Matrix3d homography[3];

    for (int i = 0; i < kSearchFrames; ++i) {
      if (i >= frame_num)
        continue;
      const Frame& f2 = map->frames[frame_num];
      const Frame& f1 = map->frames[frame_num - i - 1];
      ComputeHomography(f1, f2, &homography[i]);

      cout << homography[i] << endl;
    }

    int searched = 0;
    int updated = 0;
    for (auto& point : map->points) {
      if (point.bad_)
        continue;
      int s = frame_num - point.last_frame();
      if (s > kSearchFrames)
        continue;

      CHECK_GT(3, s);

      ++searched;

      CHECK_LT(frame_num - point.last_frame() - 1, kSearchFrames);
      CHECK_GE(frame_num - point.last_frame() - 1, 0);
      const Matrix3d& homog = homography[frame_num - point.last_frame() - 1];

      Vector2d location = ComputePoint(point.last_point(), homog);
      Vector2d lp = point.last_point();
      location = lp;

      FPos fp = curr->UpdatePosition(
          *prev[s - 1],
          vectorToFPos(location),
          vectorToFPos(lp));
      if (fp.isInvalid())
        continue;

      Observation o;
      o.frame_ref = frame_num;
      o.pt = fposToVector(fp);
      point.observations_.push_back(o);
      ++updated;
    }
#if 0
    for (int i = 0; i < 4; ++i) {
      printf("Octave %d\n", i);
      for (int j = 0; j < OctaveSet::kHistSize; ++j) {
        printf("%2d: %5d %5d\n",
               j, curr->fwd_hist[i][j], curr->rev_hist[i][j]);
      }
    }
#endif


    printf("Searched %d, updated %d\n", searched, updated);
    return 0;
  }

  void FindNewCorners(LocalMap* map, int frame_num) {
    // Mask out areas where we already have corners.
    const int grid_size = 24;
    Grid grid(grid_size, grid_size);
    int valid_points = 0;
    int sectors_marked = 0;
    for (auto& point : map->points) {
      if ((frame_num - point.last_frame()) > 1)
        continue;
      if (point.bad_)
        continue;
      FPos fp(vectorToFPos(point.last_point()));

      int count = grid.groupmark(grid_size * fp.x, grid_size * fp.y);
      sectors_marked += count;
      ++valid_points;
    }

    printf("Valid: %d, sectors %d\n", valid_points, sectors_marked);


    int found = 0;
    FPos fgrid_size(1. / grid_size, 1. / grid_size);

    for (auto& p : grid) {
      FPos corner((float)p.x / grid_size, (float) p.y / grid_size);
      FRegion fregion(corner, corner + fgrid_size);
      FPos fp = curr->SearchBestCorner(fregion, 0);

      if (fp.isInvalid())
        continue;

      int score = curr->CheckCorner(fp);
     // printf("score: %d\n", score);
      // TODO: Extract magic.
      if (score < 2000)
        continue;

      map->points.push_back(TrackedPoint());

      TrackedPoint& point = map->points.back();
      Observation o;
      o.frame_ref = frame_num;
      o.pt = fposToVector(fp);
      point.observations_.push_back(o);
      found++;
      grid.groupmark(p.x, p.y);
    }
    printf("Search found %d\n", found);

   // LOG("Now tracking %d corners\n", cset.access().num_valid());
  }


  int ProcessFrame(uint8_t* data, int width, int height, LocalMap* map) {
    int frame_num = map->AddFrame();
    curr->FillOctaves(data, width, height);
    // fill_pose(cimage->pose());

    UpdateCorners(map, frame_num);
    FindNewCorners(map, frame_num);

    flip();

    return frame_num;
  }

  void flip() {

    auto t = prev[kSearchFrames - 1];
    for (int i = 1; i < kSearchFrames; ++i) {
      prev[i] = prev[i - 1];
    }
    prev[0] = curr;
    curr = t;
  }
  static const int kSearchFrames = 1;

  OctaveSet* curr;
  OctaveSet* prev[kSearchFrames];
};


void DrawCross(cv::Mat* out, const Vector2d& point, int size, Scalar color) {
  Point2f a(
      (point(0) + 1)/2 * out->cols,
      (point(1) + 1)/2 * out->rows);
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


int main(int argc, char*argv[]) {
  if (argc < 2) {
    fprintf(stderr, "Usage: slam filename\n");
    exit(1);
  }

  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.

  cv::VideoCapture vid(argv[1]);
  cv::Mat img;
  cv::Mat t;
  cv::Mat out;
  cv::Mat grey;

  Ptr<FeatureDetector> detector =
      new GridAdaptedFeatureDetector(
          new PyramidAdaptedFeatureDetector(
              FeatureDetector::create("FAST")),
              150);
  ORB extractor(150, 1.3, 3);

  int frame = -1;

  LocalMap map;
  Tracking tracking;

  Slam slam;
  while (vid.read(img)) {
    frame++;

    cv::transpose(img, t);
    cv::resize(t, out, Size(t.cols / 2, t.rows / 2));
    cv::cvtColor(t, grey, CV_RGB2GRAY);

    int frame_num = tracking.ProcessFrame((uint8_t*)grey.data,
                                          grey.cols,
                                          grey.rows,
                                          &map);

    CHECK_EQ(frame_num, frame);


    //UpdateMap(&map, frame, normed_points, descriptors);

    for (const auto& point : map.points) {
      if (point.last_frame() != frame)
        continue;
      if (point.num_observations() == 1) {
        DrawCross(&out, point.last_point(), 2, Scalar(0,255,0));
        continue;
      }
      int num = point.observations_.size();
      for (int i = 0; i < (num - 1); ++i) {
        DrawLine(&out, point.observations_[i].pt,
            point.observations_[i+1].pt, Scalar(0,0,0));
      }
      DrawCross(&out, point.last_point(), 5, Scalar(0,0,255));
    }

    cv::imshow("Display window", out);

    int mod = 5;

    slam.Run(&map, frame - 2, false);
    slam.ReprojectMap(&map);
    map.Clean();

    if ((frame % mod) == 0) {
      slam.Run(&map, -1, false);
      slam.ReprojectMap(&map);
      map.Clean();

      //DumpMap(&map);
      //cv::waitKey(0);
    }
    if (frame >= 100)
      break;
    //cv::waitKey(0);
  }
  //slam.Run(&map, -1, true);

  DumpMap(&map);

  printf("Iterations: %d, error %f\n",
         slam.iterations(),
         slam.error());
  printf("width %d, height %d\n", grey.cols, grey.rows);
  return 0;
}
