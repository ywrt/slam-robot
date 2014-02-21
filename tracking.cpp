/*
 * tracking.cpp
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <glog/logging.h>

#include "localmap.h"
#include "octaveset.h"
#include "slam.h"
#include "grid.h"
#include "histogram.h"

#include "tracking.h"



Tracking::Tracking() :
curr(new OctaveSet),
prev {NULL, } {
  for (int i = 0; i < kSearchFrames; ++i) {
    prev[i] = new OctaveSet;
  }
}

namespace {

void ComputeHomography(const Pose& f1, const Pose& f2, Matrix3d* homog) {
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

}  // namespace

int Tracking::UpdateCorners(LocalMap* map, int frame_num) {
  Matrix3d homography[3];

  for (int i = 0; i < kSearchFrames; ++i) {
    if (i >= frame_num)
      continue;
    const Pose& f2 = map->frames[frame_num];
    const Pose& f1 = map->frames[frame_num - i - 1];
    ComputeHomography(f1, f2, &homography[i]);

    cout << homography[i] << endl;
  }

  int searched = 0;
  int updated = 0;
  for (auto& point : map->points) {
    if (point->bad_)
      continue;
    int s = frame_num - point->last_frame();
    if (s > kSearchFrames)
      continue;

    CHECK_GT(3, s);

    ++searched;

    CHECK_LT(frame_num - point->last_frame() - 1, kSearchFrames);
    CHECK_GE(frame_num - point->last_frame() - 1, 0);
    const Matrix3d& homog = homography[frame_num - point->last_frame() - 1];

    Vector2d location = ComputePoint(point->last_point(), homog);
    Vector2d lp = point->last_point();
    location = lp;

    FPos fp = curr->UpdatePosition(
        *prev[s - 1],
        vectorToFPos(location),
        vectorToFPos(lp));
    if (fp.isInvalid())
      continue;

    Observation o;
    o.frame_idx = frame_num;
    o.pt = fposToVector(fp);
    point->observations_.push_back(o);
    ++updated;
  }
#if 0
for (int i = 0; i < 4; ++i) {
  printf("Octave %d\n", i);
  cout << "fwd\n" << curr->fwd_hist[i].str();
  cout << "rev\n" << curr->rev_hist[i].str();
}
#endif


printf("Searched %d, updated %d\n", searched, updated);
return 0;
}

void Tracking::FindNewCorners(LocalMap* map, int frame_num) {
  // Mask out areas where we already have corners.
  const int grid_size = 24;
  Grid grid(grid_size, grid_size);
  int valid_points = 0;
  int sectors_marked = 0;
  for (auto& point : map->points) {
    if ((frame_num - point->last_frame()) > 1)
      continue;
    if (point->bad_)
      continue;
    FPos fp(vectorToFPos(point->last_point()));

    int count = grid.groupmark(grid_size * fp.x, grid_size * fp.y);
    sectors_marked += count;
    ++valid_points;
  }

  printf("Valid: %d, sectors %d\n", valid_points, sectors_marked);


  int found = 0;
  FPos fgrid_size(1. / grid_size, 1. / grid_size);

  static Histogram score_hist(20, 1000);

  for (auto& p : grid) {
    FPos corner((float)p.x / grid_size, (float) p.y / grid_size);
    FRegion fregion(corner, corner + fgrid_size);
    FPos fp = curr->SearchBestCorner(fregion, 0);

    if (fp.isInvalid())
      continue;

    int score = curr->CheckCorner(fp);
    score_hist.add(score);

    // TODO: Extract magic.
    if (score < 5000)
      continue;

    TrackedPoint* point = map->AddPoint();

    Observation o;
    o.frame_idx = frame_num;
    o.pt = fposToVector(fp);
    point->observations_.push_back(o);
    found++;
    grid.groupmark(p.x, p.y);
  }
  printf("Search found %d\n", found);

  cout << score_hist.str() << "\n";
  // LOG("Now tracking %d corners\n", cset.access().num_valid());
}


int Tracking::ProcessFrame(uint8_t* data, int width, int height, LocalMap* map) {
  int frame_num = map->AddFrame();
  curr->FillOctaves(data, width, height);
  // fill_pose(cimage->pose());

  UpdateCorners(map, frame_num);
  FindNewCorners(map, frame_num);

  flip();

  return frame_num;
}

void Tracking::flip() {
  auto t = prev[kSearchFrames - 1];
  for (int i = 1; i < kSearchFrames; ++i) {
    prev[i] = prev[i - 1];
  }
  prev[0] = curr;
  curr = t;
}
