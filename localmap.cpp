/*
 * localmap.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */
#include <map>
#include <iostream>
#include <glog/logging.h>
#include "histogram.h"

#include "project.h"

#include "localmap.h"

// Project a point in worldspace into Frame pixel space.
bool Frame::Project(const Vector4d& point, Vector2d* result) const {
  ProjectPoint project;
  return project(
          pose.rotation(),
          pose.translation(),
          point.data(),
          result->data());
}

Vector4d Frame::Unproject(const Vector2d& point, double distance) const {
  Vector4d result;
  result.head<2>() = point * distance;
  result(2) = distance;
  result(3) = 1;

  result.head<3>() = (pose.rotation_.inverse() * (result.head<3>() - pose.translation_)).eval();
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
    curr->pose.translation()[0] += 150;  // Assume a small movement in X.
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

TrackedPoint* LocalMap::AddPoint(int id, const Vector4d& location) {
  std::unique_ptr<TrackedPoint> p(new TrackedPoint);
  p->location_ = location;
  p->id_ = id;
  points.push_back(std::move(p));
  return points.back().get();
}

void LocalMap::Normalize() {
  if (frames.size() < 2)
    return;

  printf("Normalize\n");
  auto& pose1 = frames[0]->pose;
  auto& pose2 = frames[1]->pose;
#if 1
  // Re-orient the map back to baseline, and base scale.
  auto xlate = pose1.rotation_ * (-pose1.translation_).eval();
  // The distance between the first two frames should be 150mm.
  double scale = 150. / (pose1.rotation_.inverse() * pose1.translation_ - pose2.rotation_.inverse() * pose2.translation_).norm();
  // First translate back to origin.
  for (auto& f : frames) {
    auto& t = f->pose.translation_;
    auto& r = f->pose.rotation_;
    t += r * xlate;
    t *= scale;
    printf("{%9.4f, %9.4f, %9.4f}\n", t[0], t[1], t[2]);
  }
  for (auto& p : points) {
    // normalize to world co-ordinates.
    auto& loc = p->location_;
    loc.head<3>() -= xlate * loc[3];
    loc[3] /= scale;
  }
#endif
#if 0
  // Then rotate, first bring frame 0 to the identity rotation, and then
  // rotating to bring the second frame to [0,150,0]
  auto rotate = frames[0]->pose.rotation_.inverse().matrix().eval();
  rotate = (Quaterniond().setFromTwoVectors(pose2.rotation_.inverse() * frames[1]->pose.translation_, -Vector3d::UnitY()) * rotate).eval();

  auto inverse = rotate.inverse().eval();
  for (auto& f : frames) {
    auto& r = f->pose.rotation_;
    //f->pose.translation_ = (r.inverse() * rotate * r * f->pose.translation_).eval();
    r = (r * rotate).eval();
  }
  for (auto& p : points) {
    // Don't need to norm as we're only rotating.
    auto& loc = p->location_;
    loc.head<3>() = (inverse * loc.head<3>()).eval();
  }
#endif
}

bool LocalMap::Clean(double error_threshold) {
  printf("Cleaning\n");

  std::multimap<double, TrackedPoint*> errmap;

  for (auto& point : points) {
    point->location_.normalize();

    if (point->num_observations() < 2)
     continue;

    auto& o = point->observations_.back();
    if (o.frame_idx < 0)
      continue;

    double err = o.error.norm() * 1000;

    if (err > error_threshold) {
      errmap.insert({err, point.get() });
    }
  }
  
  if (!errmap.size())
    return true;

  double maxerr = errmap.rbegin()->first;
  maxerr = max(error_threshold, maxerr / 4.);
  cout << "Maxerr set to " << maxerr << "\n";
  bool result = true;
  for (auto iter = errmap.rbegin() ; iter != errmap.rend(); ++iter) {
    if (iter->first < maxerr)
      break;
    TrackedPoint* point = iter->second;
    auto& o = point->observations_.back();  // Observation.

    // Debug dump.
    printf("f %3d, p %3d : (matches %d) [%7.3f %7.3f] err*1000 [%7.2f,%7.2f] -> %.2f [%7.3f, %7.3f, %7.3f]\n",
       o.frame_idx,
       point->id_,
       point->num_observations(),
       o.pt(0), o.pt(1),
       o.error(0) * 1000, o.error(1) * 1000,
       iter->first, // error
       point->location()[0] / point->location()[3],
       point->location()[1] / point->location()[3],
       point->location()[2] / point->location()[3]
      );

    if (o.frame_idx < 0)
      continue;  // Already disabled.

    // Disable point from being considered by slam.
    o.frame_idx = -o.frame_idx; // Mark as bad and ignore it in future.
    point->bad_++;
    result = false;
  }

  return result;
}

void LocalMap::Stats() {
  Histogram err_hist(20);

  printf("Stats\n");
  for (auto& point : points) {
    point->location_.normalize();
    if (point->num_observations() < 2)
     continue;
    for (auto& o : point->observations_) {
      double err = o.error.norm() * 1000;
      if (err < 50 && o.frame_idx < 0) {
        // Debug dump.
        printf("f %3d, p %3d : (matches %d) [%7.3f %7.3f] err*1000 [%7.2f,%7.2f] -> %.2f [%7.3f, %7.3f, %7.3f]\n",
           o.frame_idx,
           point->id_,
           point->num_observations(),
           o.pt(0), o.pt(1),
           o.error(0) * 1000, o.error(1) * 1000,
           err, // error
           point->location()[0] / point->location()[3],
           point->location()[1] / point->location()[3],
           point->location()[2] / point->location()[3]
          );
        if (err < 2.) {
          o.frame_idx = -o.frame_idx;  // Restore it
          point->bad_--;
        }

      }
      if (o.frame_idx < 0)
        continue;
      err_hist.add(err);
    }
  }
  cout << "LocalMap Error histogram\n" << err_hist.str();

  for (unsigned int i = 0 ; i < frames.size(); ++i) {
    auto f = frames[i].get();

    double distance = 0;
    if (i > 0) {
      auto prev = frames[i-1].get();
      distance = (f->pose.translation_ - prev->pose.translation_).norm();
    }
    printf("Frame %3d : [ % 9.4f, % 9.4f, % 9.4f ] distance %6.4f\n",
        f->frame_num,
        f->pose.translation_[0],
        f->pose.translation_[1],
        f->pose.translation_[2],
        distance);
  }
}
