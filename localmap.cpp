/*
 * localmap.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */
#include <iostream>
#include <glog/logging.h>
#include "histogram.h"

#include "project.h"

#include "localmap.h"

// Project a point in worldspace into Frame pixel space.
bool Frame::Project(const Vector4d& point, Vector2d* result) const {
  ProjectPoint project;
  return project(
          camera->xyscale(),
          camera->distortion(),
          pose.rotation(),
          pose.translation(),
          point.data(),
          result->data());
}

Vector4d Frame::Unproject(const Vector2d& point, double distance) const {
  Vector4d result;
  result(0) = point(0) / camera->xyscale()[0] * distance;
  result(1) = point(1) / camera->xyscale()[1] * distance;
  result(2) = distance;
  result(3) = 1;

  auto r = pose.rotation_.inverse() * (result.topLeftCorner(3,1) - pose.translation_);
  result.topLeftCorner(3,1) = r;
  result.normalize();
  return result;
}


Frame* LocalMap::AddFrame(Camera* cam) {
  std::unique_ptr<Frame> p(new Frame(frames.size(), cam));
  frames.push_back(std::move(p));
  return frames.back().get();
}

Camera* LocalMap::AddCamera() {
  std::unique_ptr<Camera> p(new Camera);
  cameras.push_back(std::move(p));
  return cameras.back().get();
}

void LocalMap::EstimateMotion(const Frame* f2, const Frame* f1, Frame* curr) {
  if (!f1 && !f2) {
    curr->pose = Pose();
    return;
  }

  if (!f2) {  // Only 1 previous frame.
    curr->pose = f1->pose;
    curr->pose.translation()[0] += 0.15;  // Assume a small movement in X.
    return;
  }

  const auto& p2 = f2->pose;
  const auto& p1 = f1->pose;

  Vector3d motion = p1.translation_ - p2.translation_;
  if (motion.norm() > 1)
    motion /= motion.norm();

  curr->pose = p1;
  curr->pose.translation_ += motion;
}

TrackedPoint* LocalMap::AddPoint(const Vector4d& location) {
  std::unique_ptr<TrackedPoint> p(new TrackedPoint);
  p->location_ = location;
  points.push_back(std::move(p));
  return points.back().get();
}

void LocalMap::Clean() {
 Histogram err_hist(20);

 int curr_frame = frames.size() - 1;
 for (auto& point : points) {
   point->location_.normalize();
   if (point->num_observations() < 2)
     continue;
   int poor_matches = 0;
   for (auto& o : point->observations_) {
     double err = o.error.norm() * 1000;
     err_hist.add(err);

     //if (err < 5)
     //  continue;
     printf("frame %3d : (matches %d) [%7.3f %7.3f] (%7.2f,%7.2f) -> %.2f\n",
         o.frame_idx,
         point->num_observations(),
         o.pt(0), o.pt(1),
         o.error(0) * 1000, o.error(1) * 1000,
         err);
     ++poor_matches;
   }
   if (poor_matches && point->last_frame() == curr_frame) {
     point->bad_ = true;
     point->observations_.pop_back();
   }
 }
 cout << "LocalMap Error histogram\n" << err_hist.str();
}
