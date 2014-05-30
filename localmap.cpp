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

// Project a point in worldspace into Frame pixel space.
bool Frame::Project(const Vector4d& point, Vector2d* result) const {
  ProjectPoint project;
  return project(
          rotation().coeffs().data(),
          translation().data(),
          camera()->k,
          point.data(),
          result->data());
}

Vector4d Frame::Unproject(const Vector2d& point, double distance) const {
  Vector4d result;
  result.head<2>() = point * distance;
  result(2) = distance;
  result(3) = 1;

  result.head<3>() = (rotation().inverse() * result.head<3>() + translation()).eval();
  result.normalize();
  return result;
}

void TrackedPoint::AddObservation(Observation* obs) {
  observations_.push_back(obs);
  CheckFlags();
}
 
void TrackedPoint::CheckFlags() {
  // Clear NO_OBSERVATIONS flag if there are multiple
  // good observations.
  if (has_flag(NO_OBSERVATIONS)) {
    int good = 0;
    for (const auto& o : observations_) {
      if (!o->enabled())
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
      if (!o->enabled())
        continue;
      if (!has_base) {
        base = o->frame->position();
        has_base = true;
        continue;
      }
      double dist = (o->frame->position() - base).norm();
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

void Frame::Commit() {
  for (const auto& obs : observations_) {
    obs->point->AddObservation(obs.get());
  }
}


Frame* LocalMap::AddFrame(Camera* cam) {
  std::unique_ptr<Frame> p(new Frame(frames.size(), cam));
  if (frames.size())
    p->set_previous(frames.back().get());
  frames.push_back(std::move(p));
  return frames.back().get();
}

void LocalMap::AddCamera(Camera* cam) {
  std::unique_ptr<Camera> p(cam);  // Takes ownership.
  cameras.push_back(std::move(p));
}

TrackedPoint* LocalMap::AddPoint(int id, const Vector4d& location) {
  std::unique_ptr<TrackedPoint> p(new TrackedPoint(location, id));
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
  auto xlate = ( -pose1->translation()).eval();
  // The distance between the first two frames should be 150mm.
  double scale = 150. / (pose1->translation() - pose2->translation()).norm();
  scale = 1.;
  // First translate back to origin.
  for (auto& f : frames) {
    f->translation() += xlate;
    f->translation() *= scale;
  }

  for (auto& p : points) {
    // normalize to world co-ordinates.
    p->move(xlate);
    p->rescale(1./scale);
  }
#endif
#if 1
  // Then rotate, first bring frame 0 to the identity rotation, and then
  // rotating to bring the second frame to [0,150,0]
  auto rotate = pose1->rotation().matrix().eval();
  auto inverse = rotate.inverse().eval();
  //rotate = (Quaterniond().setFromTwoVectors(rotate * pose2.rotation_.inverse() * pose2.translation_, Vector3d::UnitX()));

  for (auto& f : frames) {
    f->rotation() = (f->rotation() * inverse).eval();
    f->translation() = (rotate * f->translation()).eval();
  }
  for (auto& p : points) {
    // Don't need to norm as we're only rotating.
    p->location().head<3>() = (rotate * p->location().head<3>()).eval(); 
  }
#endif
}

// Remove and destroy the most recent keyframe.
void LocalMap::PopFrame() {
  if (!frames.size()) return;
  Frame* f = frames.back().get();

  CHECK(!f->is_keyframe_);

  // Should be the most recent observation for these points.
  for (const auto& obs : f->observations()) {
    CHECK(obs->point->RemoveObservation(f));
  }

  // Destructors will take care of everything else.
  frames.pop_back();
}

void LocalMap::CheckNotMoving() {
  if (frames.size() < 4) return;
  int n = frames.size();
  double d1 = (frames[n-1]->position() - frames[n-3]->position()).norm();
  double d2 = (frames[n-2]->position() - frames[n-4]->position()).norm();
  if ((d1*d1 + d2*d2) > 5) return;  // Still moving.

  if (frames[n-1]->is_keyframe_) return;
  if (frames[n-2]->is_keyframe_) return;

  printf("Removing idle frames.\n");

  PopFrame();
  PopFrame();
}

void PrintObs(const Observation& o, const TrackedPoint& point) {
  // Debug dump.
  printf("f %3d, p %3d : (matches %d) [%7.3f %7.3f] err [%7.2f,%7.2f] -> %.2f [%7.3f, %7.3f, %7.3f]\n",
     o.frame->id(),
     point.id(),
     point.num_observations(),
     o.pt(0), o.pt(1),
     o.error(0), o.error(1),
     o.error.norm(),
     point.location()[0] / point.location()[3],
     point.location()[1] / point.location()[3],
     point.location()[2] / point.location()[3]
    );
}

// cam1 = R1 * w + T1 (from)
// cam2 = R2 * w + T2 (to)
//
// cam1 - T1 = R1 * w;
// R1^ (cam1 - T1) = w;
// w = R1^ (cam1 - T1)
// cam2 = R2 * R1^ * (cam1 - T1) + T2;
// cam2 = R2 * R1^ * (cam1 - T1) + R2 * R1^ * R1 * R2^ * T2;
// cam2 = R2 * R1^ * (cam1 - T1 + R1 * R2^ * T2)
// cam2 = R2 * R1^ * (cam1 - (T1 - R1 * R2^ * T2))

Matrix3d EssentialMatrix(Frame* from, Frame* to) {
  // E = R t_x
  //Matrix3d rotation = to->rotation().matrix() * from->rotation().inverse().matrix();
  //Matrix3d rotation = from->rotation().inverse().matrix() * to->rotation().matrix();
  Matrix3d rotation = to->rotation().matrix() * from->rotation().inverse().matrix();
  Vector3d translation = to->translation() - from->translation();
  translation.normalize();

  Matrix3d skew;
  skew << 0, -translation[2], translation[1],
          translation[2], 0, -translation[0],
          -translation[1], translation[0], 0;

  return rotation * skew;
}

// Check that each recently matched point obeys the epipolar constraint.
void LocalMap::ApplyEpipolarConstraint() {
  for (auto& point : points) {
    // Only check points that have been solved by SLAM.
    if (point->num_observations() < 2)
      continue;
    if (!point->feature_usable())
      continue;
    if (point->has_flag(TrackedPoint::Flags::BAD_FEATURE))
      continue;

    Observation* obs1 = point->observation(-1);
    Observation* obs2 = point->observation(-2);
    for (int i = 3;
        i < point->num_observations() &&
        //(obs1.frame->camera() == obs2.frame->camera() ||
        (obs2->disabled()); ++i) {
      obs2 = point->observation(-i);
    }
    if (obs1->frame->camera() == obs2->frame->camera() || obs2->disabled())
      continue;

    Vector2d p1 = obs1->frame->camera()->PixelToPlane(obs1->pt);
    Vector2d p2 = obs2->frame->camera()->PixelToPlane(obs2->pt);
    Vector3d h1(p1(0),p1(1), 1);
    Vector3d h2(p2(0),p2(1), 1);

    Matrix3d e = EssentialMatrix(obs1->frame, obs2->frame);

    double threshold = 0.0015;

    double r = h2.transpose() * e * h1;
    printf("%c p %3d: r = %7.4f f%3d -> f%3d : dist %9.4f\n",
        (fabs(r) > threshold) ? '*' : ' ',
        point->id(), r,
        obs1->frame->id(), obs2->frame->id(), point->position()[2]);
    if (fabs(r) > threshold * 100) {
      if (point->num_observations() > 8) {
        obs1->disable();
        point->set_flag(TrackedPoint::Flags::MISMATCHED);
      } else {
        point->set_flag(TrackedPoint::Flags::BAD_FEATURE);
      }
    }
  }
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
    if (point->location()[3] < 0) {
      point->location()[3] = -point->location()[3];
      printf(" loc was -ve\n");
    }
    if (fabs(point->location()[3]) < 1e-6) {
      point->location()[3] = 1e-6;
      printf(" loc was small\n");
    }

    double sum_err = 0;
    for (auto& o : point->observations()) {
      // Is the reproject error too large? if so, maybe disable the obs.
      // Is the reproject error too small to remain disabled? If so, enable.
      double err = o->error.norm();
      sum_err += err;
      // TODO: Fix re-enabling. It conflicts with points disabled due to the
      // epipolar constraint.
      if (!o->enabled() && err < error_threshold * 0.75 && 0) {
        // Hmmm. Observation now has small error. Restore it.
        o->enable();
        changed_points.insert(point.get());
        printf("p %3d, f %3d: Re-enabled point.\n", point->id(), o->frame->id());
        PrintObs(*o, *point);
      } 

      // Is the point too close to the camera? This is an indication of a badly 
      // tracked point (bundle adjustment is bringing it singular).
      // First, move the point into frame space.
      // if (o->frame->position() - point->position()).norm() < 1) {
      Vector3d pos = o->frame->rotation() * (point->position() - o->frame->translation());
      if (pos[2] < 1) {
        printf("p %3d, f %3d: Point is too close to camera. %f\n", point->id(), o->frame->id(), pos[2]);
        point->set_flag(TrackedPoint::Flags::BAD_LOCATION);
        changed_points.insert(point.get());
        break;
      }
      if (o->enabled() && err > error_threshold) {
          // If the error exceeds the threshold, add it to the list to possibly mark as bad.
          errmap.insert({err, {point.get(), o} });
      }


    }
#if 0
    auto& obs = point->observations().back();
    double err = obs.error.norm() ;
    if (obs.enabled() && obs.error.norm() > error_threshold) {
        // If the error exceeds the threshold, add it to the list to possibly mark as bad.
        errmap.insert({err, {point.get(), &obs} });
    }
#endif

    double avg_err = sum_err / point->num_observations();
    if (avg_err > 1.5 && point->num_observations() > 4) {
      printf("p %3d: Bad feature\n", point->id());
      point->set_flag(TrackedPoint::Flags::BAD_FEATURE);
      changed_points.insert(point.get());
    }

    point->set_uncertainty(avg_err);
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
  int slam(0), no_base(0), no_obs(0), bad_loc(0), bad_feat(0);
  for (auto& point : points) {
    if (point->slam_usable()) {
      ++slam;
    }
    if (point->has_flag(TrackedPoint::Flags::NO_BASELINE))
      ++no_base;
    if (point->has_flag(TrackedPoint::Flags::NO_OBSERVATIONS))
      ++no_obs;
    if (point->has_flag(TrackedPoint::Flags::BAD_FEATURE))
      ++bad_feat;
    if (point->has_flag(TrackedPoint::Flags::BAD_LOCATION))
      ++bad_loc;

    if (point->has_flag(TrackedPoint::Flags::NO_BASELINE))
      continue;

    auto loc = point->position();
    printf("p %3d (%d matches) [%9.2f, %9.2f, %9.2f] %s%s%s%s%s%s\n",
        point->id(), point->num_observations(),
        loc[0], loc[1], loc[2],
        point->slam_usable() ? "slam " : "",
        point->has_flag(TrackedPoint::Flags::BAD_LOCATION) ? "BAD_LOCATION " : "",
        point->has_flag(TrackedPoint::Flags::NO_BASELINE) ? "NO_BASELINE " : "",
        point->has_flag(TrackedPoint::Flags::NO_OBSERVATIONS) ? "NO_OBSERVATIONS " : "",
        point->has_flag(TrackedPoint::Flags::MISMATCHED) ? "MISMATCHED " : "",
        point->has_flag(TrackedPoint::Flags::BAD_FEATURE) ? "BAD_FEATURE " : ""
        );
    for (auto& o : point->observations()) {
      double err = o->error.norm() ;
      printf("  f %3d: [%c] err %6.4f rad %6.1f [%8.4f, %8.4f] err [%8.4f, %8.4f]\n",
          o->frame->id(),
          o->disabled() ? 'D' : ' ',
          err,
          o->pt.norm() ,
          o->pt(0), o->pt(1),
          o->error(0) , o->error(1) 
          );
      if ((err < 50 && o->disabled()) || err > 5.) {
        // Debug dump.
        //PrintObs(o, *point);
      }
      if (!o->enabled() || !point->slam_usable()) {
        disabled_err_hist.add(err);
      } else {
        enabled_err_hist.add(err);
      }
    }
  }
  printf("%d slam points from %zd total points (%d no base, %d no obs, %d bad loc, %d bad feat)\n",
      slam, points.size(),
      no_base, no_obs, bad_loc, bad_feat);

  cout << "LocalMap Error histogram for enabled obs:\n" << enabled_err_hist.str();
  cout << "LocalMap Error histogram for disabled obs:\n" << disabled_err_hist.str();

  for (unsigned int i = 0 ; i < frames.size(); ++i) {
    auto f = frames[i].get();

    double distance = 0;
    double ddist = 0;
    auto pos = f->position();
    if (i > 0) {
      distance = (pos - frames[i-1]->position()).norm();
    }
    if (i > 1) {
      ddist = (pos - frames[i-2]->position()).norm();
    }

    auto rot = f->rotation();
    printf("Frame %3d : [ % 9.4f, % 9.4f, % 9.4f ] distance %8.1f ddist %8.1f [%f,%f,%f,%f]\n",
        f->id(),
        pos(0), pos(1), pos(2),
        distance,
        ddist,
        rot.w(), rot.x(), rot.y(), rot.z()
        );
  }
}

