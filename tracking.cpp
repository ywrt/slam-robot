/*
 * tracking.cpp
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#include <eigen3/Eigen/Eigen>
#include <glog/logging.h>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

#include "grid.h"
#include "histogram.h"
#include "imagedata.h"
#include "localmap.h"
#include "octaveset.h"
#include "slam.h"

#include "tracking.h"


Tracking::Tracking() {}
Tracking::~Tracking() {}

namespace {

Matrix3d ComputeHomography(const Pose& f1, const Pose& f2) {
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

  return AngleAxisd(-M_PI/2, Vector3d::UnitZ()) *
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

int Tracking::UpdateCorners(LocalMap* map, const ImageData& curr, int frame_num, vector<Vector2d>* tracked) {
  Frame* frame = map->frame(frame_num);

  int searched = 0, updated = 0;
  for (auto& point : map->points) {
    auto obs = point->last_obs();
    auto iter = data_.find(obs.frame_idx);
    if (iter == data_.end())
      continue;  // Don't have any patch data.
    
    Vector2d p;
    if (!frame->Project(point->location(), &p))
      continue;  // Behind the camera.
    if (p.lpNorm<Infinity>() > 1)
      continue;  // out of frame.


    auto ref = iter->second.get();  // ImageData*

    auto homography = ComputeHomography(map->frame(obs.frame_idx)->pose, frame->pose);

    // TODO: Extract patch from 'ref'. patch source shape is
    // computed using homography.

    auto pt = obs.pt;
    ++searched;
    if (!ref->UpdateCorner(curr, homography, 5000, &pt))
      continue;
  
    point->AddObservation({p, frame_num});
    tracked->push_back(p);
    ++updated;
  }

  printf("Searched %d, updated %d\n", searched, updated);
  return 0;
}

void Tracking::FindNewCorners(const Mat& image, const vector<Vector2d>& tracked) {
  // Mask out areas where we already have corners.
  const int grid_size = 24;
  Grid grid(grid_size, grid_size);

  int valid_points = 0;
  int sectors_marked = 0;
  for (const auto& point : tracked) {
    int x = 0.5 + (point(0) + 1) * 0.5 * grid_size;
    int y = 0.5 + (point(1) + 1) * 0.5 * grid_size;
    int count = grid.groupmark(x, y);
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

    Vector2d frame_point = fposToVector(fp);
    Vector4d location = map->frame(frame_num)->Unproject(frame_point, 5);
    TrackedPoint* point = map->AddPoint(location);

    Observation o;
    o.frame_idx = frame_num;
    o.pt = frame_point;
    point->observations_.push_back(o);
    found++;
    grid.groupmark(p.x, p.y);
  }
  printf("Search found %d\n", found);

  cout << score_hist.str() << "\n";
  // LOG("Neliminatingow tracking %d corners\n", cset.access().num_valid());
}


int Tracking::ProcessFrame(const cv::Mat& image, LocalMap* map) {
  Frame* frame = map->AddFrame(map->cameras[0].get());

  std::unique_ptr<ImageData> curr(new ImageData(image));
  // fill_pose(cimage->pose());

  vector<Vector2d> tracked;
  UpdateCorners(map, *(curr.get()), frame->frame_num, &tracked);
  FindNewCorners(map, frame->frame_num);

  data_[frame->frame_num] = std::move(curr);
  if (data_.size() > 4) {
    data_.erase(data_.begin());  // erase the lowest frame_num.
  }
  return frame->frame_num;
}
