/*
 * localmap.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */
#include <map>
#include <set>
#include <iostream>
#include <glog/logging.h>
#include "histogram.h"

#include "project.h"

#include "localmap.h"


// Takes frame coordinates and maps to (distorted) pixel coordinates.
Vector2d Camera::Distort(const Vector2d& px) const {
  double x, y;
  double x0 = x = px[0];
  double y0 = y = px[1];

  // k1, k2, p1, p2, k3
  // compensate distortion iteratively
  for( unsigned int j = 0; j < 5; j++ ) {
    double r2 = x*x + y*y;

    double icdist = 1./(1 + ((k3*r2 + k2)*r2 + k1)*r2);
    double deltaX = 2*p1*x*y + p2*(r2 + 2*x*x);
    double deltaY = p1*(r2 + 2*y*y) + 2*p2*x*y;
    x = (x0 - deltaX)*icdist;
    y = (y0 - deltaY)*icdist;
  }

  // Save undistorted pixel coords:
  Vector2d result;
  result[0] = x;
  result[1] = y;
  return result.array() * focal.array() + center.array();
}

// Takes pixel co-ordinates and returns undistorted frame coordinates.
Vector2d Camera::Undistort(const Vector2d& px) const {
  // TODO: Actually do the distortion.
  Vector2d p = px;
  p -= center;
  p.array() /= focal.array();

  double rd = p.squaredNorm();
  double xd = p(0);
  double yd = p(1);

  double xu = xd*(1+k1*rd + k2*rd*rd + k3*rd*rd*rd) + 2*p1*xd*yd + p2*(rd+ 2*xd*xd);
  double yu = yd*(1+k1*rd + k2*rd*rd + k3*rd*rd*rd) + p1*(rd + 2*yd*yd) + 2*p2*xd*yd;

  Vector2d result;
  result << xu, yu;
  return result;
}

// Project a point in worldspace into Frame pixel space.
bool Frame::Project(const Vector4d& point, Vector2d* result) const {
  ProjectPoint project;
  return project(
          rotation().coeffs().data(),
          translation().data(),
          point.data(),
          result->data());
}

Vector4d Frame::Unproject(const Vector2d& point, double distance) const {
  Vector4d result;
  result.head<2>() = point * distance;
  result(2) = distance;
  result(3) = 1;

  result.head<3>() = (rotation().inverse() * (result.head<3>() - translation())).eval();
  result.normalize();
  return result;
}

void TrackedPoint::AddObservation(const Observation& obs) {
  observations_.push_back(obs);
  CheckFlags();
}
 
void TrackedPoint::CheckFlags() {
  // Clear NO_OBSERVATIONS flag if there are multiple
  // good observations.
  if (has_flag(NO_OBSERVATIONS)) {
    int good = 0;
    for (const auto& o : observations_) {
      if (!o.enabled())
        continue;
      ++good;
      if (good >= 2) {
        clear_flag(NO_OBSERVATIONS);
        printf("p %3d: Cleared NO_OBSERVATIONS (%d obs)\n", id(), num_observations());
        break;
      }
    }
  }

  // TODO: Clear the NO_BASELINE flag
  if (has_flag(NO_BASELINE)) {
    Vector3d base;
    bool has_base = false;
    for (const auto& o : observations_) {
      if (!o.enabled())
        continue;
      if (!has_base) {
        base = o.frame->position();
        has_base = true;
        continue;
      }
      double dist = (o.frame->position() - base).norm();
      // TODO: Lift out constant: minimum baseline distance.
      if (dist < 50) {
        continue;
      }

      clear_flag(NO_BASELINE);
      printf("p %3d: Cleared NO_BASELINE (%d obs)\n", id(), num_observations());
      break;
    }
  }
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

TrackedPoint* LocalMap::AddPoint(int id, const Vector4d& location) {
  std::unique_ptr<TrackedPoint> p(new TrackedPoint);
  p->location_ = location;
  p->id_ = id;
  p->set_flag(TrackedPoint::Flags::NO_OBSERVATIONS);
  p->set_flag(TrackedPoint::Flags::NO_BASELINE);
  points.push_back(std::move(p));
  return points.back().get();
}

void LocalMap::Normalize() {
  if (frames.size() < 2)
    return;

  printf("Normalize\n");
  auto& pose1 = frames[0];
  auto& pose2 = frames[1];
#if 1
  // Re-orient the map back to baseline, and base scale.
  auto xlate = (pose1->rotation() * -pose1->translation()).eval();
  // The distance between the first two frames should be 150mm.
  double scale = 150. / (pose1->rotation().inverse() * pose1->translation() - pose2->rotation().inverse() * pose2->translation()).norm();
  // First translate back to origin.
  for (auto& f : frames) {
    f->translation() += f->rotation() * xlate;
    f->translation() *= scale;
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
  auto rotate = pose1.rotation_.inverse().matrix().eval();
  rotate = (Quaterniond().setFromTwoVectors(rotate * pose2.rotation_.inverse() * pose2.translation_, -Vector3d::UnitX()));

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


void PrintObs(const Observation& o, const TrackedPoint& point) {
  // Debug dump.
  printf("f %3d, p %3d : (matches %d) [%7.3f %7.3f] err*1000 [%7.2f,%7.2f] -> %.2f [%7.3f, %7.3f, %7.3f]\n",
     o.frame->id(),
     point.id(),
     point.num_observations(),
     o.pt(0), o.pt(1),
     o.error(0) * 1000, o.error(1) * 1000,
     o.error.norm() * 1000,
     point.location()[0] / point.location()[3],
     point.location()[1] / point.location()[3],
     point.location()[2] / point.location()[3]
    );
}

//
// Invalidate observations that exceed the error threshold.
// Invalidate points that are very close to frame poses.
//
bool LocalMap::Clean(double error_threshold) {
  printf("Cleaning\n");
  bool result = true;

  std::multimap<double, pair<TrackedPoint*, Observation*> > errmap;

  // Set of points that will need their flags re-checked.
  std::set<TrackedPoint*> changed_points;

  // Search for observations with high error.
  for (auto& point : points) {
    // Only check points that have been solved by SLAM.
    if (!point->slam_usable())
      continue;

    // Force point scale factor to be strictly positive.
    if (point->location_[3] < 0) {
      point->location_[3] = -point->location_[3];
    }
    if (point->location_[3] < 1e-6) {
      point->location_[3] = 1e-6;
    }

    for (auto& o : point->observations()) {
      // Is the reproject error too large? if so, maybe disable the obs.
      // Is the reproject error too small to remain disabled? If so, enable.
      double err = o.error.norm() * 1000;
      if (!o.enabled() && err < error_threshold * 0.75) {
        // Hmmm. Observation now has small error. Restore it.
        o.enable();
        changed_points.insert(point.get());
        printf("p %3d, f %3d: Re-enabled point.\n", point->id(), o.frame->id());
        PrintObs(o, *point);
      } else if (o.enabled() && err > error_threshold) {
        // If the error exceeds the threshold, add it to the list to possibly mark as bad.
        errmap.insert({err, {point.get(), &o} });
      }

      // Is the point too close to the camera? This is an indication of a badly 
      // tracked point (bundle adjustment is bringing it singular).
      // First, move the point into frame space.
      // if (o.frame->position() - point->position()).norm() < 1) {
      Vector3d pos = o.frame->rotation() * point->position() + o.frame->translation();
      if (pos[2] < 1) {
        printf("p %3d, f %3d: Point is too close to camera. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", point->id(), o.frame->id());
        point->set_flag(TrackedPoint::Flags::BAD_LOCATION);
        changed_points.insert(point.get());
        break;
      }
    }
  }
  
  // Mark bad observations in order from worst to best, stopping when the
  // current error is less than 1/4 of the maximum error.
  if (errmap.size()) {
    double maxerr = errmap.rbegin()->first;
    // TODO: Lift out constant '4.'
    maxerr = max(error_threshold, maxerr / 4.);
    cout << "Maxerr set to " << maxerr << "\n";
    for (auto iter = errmap.rbegin() ; iter != errmap.rend(); ++iter) {
      if (iter->first < maxerr)
        break;  // All remaining points have less than maxerr.

      TrackedPoint* point = iter->second.first;
      auto& o = *(iter->second.second);

      // Debug dump.
      PrintObs(o, *point);

      if (o.disabled())
        continue;  // Already disabled.

      // Disable point from being considered by slam.
      o.disable(); // Mark as bad and ignore it in future.
      point->set_flag(TrackedPoint::Flags::MISMATCHED);
      changed_points.insert(point);
      result = false;  // Observations have been removed from problem.
    }
  }

  for (auto* point : changed_points) {
    // Set error flags.
    point->set_flag(TrackedPoint::Flags::NO_OBSERVATIONS);
    point->set_flag(TrackedPoint::Flags::NO_BASELINE);
    // Clear error flags that are inappropriately set.
    point->CheckFlags();
  }

  return result;
}

void LocalMap::Stats() const {
  Histogram enabled_err_hist(10);
  Histogram disabled_err_hist(10);

  printf("Stats\n");
  int non_slam(0);
  for (auto& point : points) {
    point->location_.normalize();
    if (!point->slam_usable()) {
      ++non_slam;
      continue;
    }
    for (auto& o : point->observations()) {
      double err = o.error.norm() * 1000;
      if ((err < 50 && o.disabled()) || err > 5.) {
        // Debug dump.
        //PrintObs(o, *point);
      }
      if (!o.enabled()) {
        disabled_err_hist.add(err);
      } else {
        enabled_err_hist.add(err);
      }
    }
  }
  printf("%d non-slam points from %zd total points\n", non_slam, points.size());

  cout << "LocalMap Error histogram for enabled obs:\n" << enabled_err_hist.str();
  cout << "LocalMap Error histogram for disabled obs:\n" << disabled_err_hist.str();

  for (unsigned int i = 0 ; i < frames.size(); ++i) {
    auto f = frames[i].get();

    double distance = 0;
    auto pos = f->position();
    if (i > 0) {
      distance = (pos - frames[i-1]->position()).norm();
    }

    printf("Frame %3d : [ % 9.4f, % 9.4f, % 9.4f ] distance %6.4f\n",
        f->id(),
        pos(0), pos(1), pos(2),
        distance);
  }
}

