/*
 * localmap.h
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

// TODO: Use graph center (aka Jordan Center) to place the reference frame for the tracked points.
//

#ifndef LOCALMAP_H_
#define LOCALMAP_H_
#include <memory>
#include <vector>
#include <string>
#include <array>

#include <eigen3/Eigen/Eigen>

using namespace Eigen;
using namespace std;

class TrackedPoint;

// Camera intrinsics.
struct Camera {
  // Takes frame coordinates and maps to (distorted) pixel coordinates.
  Vector2d Distort(const Vector2d& px) const;

  // Takes pixel co-ordinates and returns undistorted frame coordinates.
  Vector2d Undistort(const Vector2d& px) const;

  double kinit[7];  // k1, k2, k3, fx, fy, cx, cy
  double k[7];  // k1, k2, k3, fx, fy, cx, cy
};

// A frame taken by a specific camera in a specific pose.
class Frame {
 public:
  Frame(int id, Camera* cam) :
    frame_id_(id),
    camera_(cam),
    rotation_{1, 0, 0, 0},
    translation_{0, 0, 0} { }

  // Project a tracked point into frame pixel space.
  bool Project(const Vector4d& point, Vector2d* result) const;

  // Convert a point in frame pixel space [-1,1]x[-1,1] and a
  // guess at distance into a homogenous point. 
  Vector4d Unproject(const Vector2d& point, double distance) const;

  // Frame origin in world space.
  Vector3d position() const {
    return rotation_.inverse() * -translation_;
  }

  // A unique id for this frame.
  int id() const { return frame_id_; }

  // Pose details.
  const Quaterniond& rotation() const { return rotation_; }
  const Vector3d& translation() const { return translation_; }
  Quaterniond& rotation() { return rotation_; }
  Vector3d& translation() { return translation_; }

  // The camera intrinsics.
  const Camera* camera() const { return camera_; }
  Camera* camera() { return camera_; }

  double dist;
 private:
  int frame_id_;
  Camera* camera_;
  Quaterniond rotation_;
  Vector3d translation_;
};


// An observation of a tracked point from a specific frame.
struct Observation {
//  Observation() : frame(nullptr), is_disabled(0) {}
  Observation(Vector2d p, Frame* frame_ptr) :
      pt(p), frame(frame_ptr), is_disabled(0) { }

  void enable() { is_disabled = 0; }
  void disable() { is_disabled = 1; }
  bool enabled() const { return !is_disabled; }
  bool disabled() const { return is_disabled; }

  Vector2d pt;  // In [-1, 1] x [-1, 1]
  Vector2d error;  // For debugging: filled in by slam.
  Frame* frame;
  int is_disabled;
};

// A fully tracked point.
// Has the location in world-space, the descriptors, and any
// observations.
//
// An impaired point can be:
//   1. Too close to a frame origin => entirely unusable.
//   2. Have too short a baseline => Not (yet) usable for SLAM solving.
//   3. Have insufficient usable observations. => Not (yet) usable for SLAM solving.
//   4. Have recent mis-matched observations. => Should no longer be attached to visual feature.
class TrackedPoint {
 public:
  TrackedPoint(const Vector4d& location, int id) :
    location_(location),
    id_(id),
    flags_(0)
    { }

  enum Flags {
    BAD_LOCATION,  // Too close to frame origin => entirely unusable.
    NO_BASELINE,  // Insufficient baseline => Not (yet) usable for SLAM.
    NO_OBSERVATIONS,  // Insufficient usable observations => Not (yet) SLAMable.
    MISMATCHED,  // Has multiple recent mis-matched obs => Detach from visual feature.
    BAD_FEATURE,  // persistent error when fitting. supress from SLAM.
  };

  // Pointer to an array of 4 doubles being [X, Y, Z, W],
  // the homogeous coordinates in world space.
  const Vector4d& location() const { return location_; }
  Vector4d& location() { return location_; }

  // The known observations of this point.
  const vector<Observation>& observations() const { return observations_; }
  vector<Observation>& observations() { return observations_; }

  // The number of observations of this point.
  int num_observations() const { return observations_.size(); }
  // Return an observation. If idx is negative, then observations relative to the
  // most recent. -1 == most recent, -2 == previous most recent, etc.
  const Observation& observation(int idx) const {
    if (idx >= 0) {
      return observations_[idx];
    } else {
      return observations_[observations_.size() + idx];
    }
  }   

  // Returns point position in world coordinates.
  Vector3d position() const {
    return location_.head<3>() / location_[3];
  }

  // Move the point by 'delta': delta is in world co-ords.
  void move(const Vector3d& delta) {
    location_.head<3>() += delta * location_[3];
  }
  void rescale(double scale) {
    location_[3] *= scale;
    location_.normalize();
  }

  // Returns the point ID (mostly used for debugging).
  int id() const { return id_; }

  // Flag handling.
  void set_flag(Flags flag) { flags_ |= (1<<flag); }
  void clear_flag(Flags flag) { flags_ &= ~(1<<flag); }
  bool has_flag(Flags flag) const { return flags_ & (1<<flag); }

  bool slam_usable() const {
    return
      !has_flag(BAD_LOCATION) &&
      !has_flag(NO_BASELINE) &&
      !has_flag(NO_OBSERVATIONS) &&
      !has_flag(BAD_FEATURE);
  }
  bool feature_usable() const { return !has_flag(MISMATCHED) & !has_flag(BAD_LOCATION); }

  // Add a new observation of this point.
  void AddObservation(const Observation& obs);

  // Clear incorrectly set flags.
  void CheckFlags();

 private:
  Vector4d location_;  // Homogeneous location in world.
  int id_;
  int flags_;
  vector<Observation> observations_;
};

//
// Description of the known world.
// 
struct LocalMap {
  // Add a new (empty) frame to the map.
  // Returns the frame number.
  Frame* AddFrame(Camera* cam);

  // Add a new camera to the map
  void AddCamera(Camera* cam);

  Frame* frame(int frame_idx) { return frames[frame_idx].get(); }
  const Frame* frame(int frame_idx) const { return frames[frame_idx].get(); }

  // Do simple motion estimation from the previous two frames.
  void EstimateMotion(const Frame* f2, const Frame* f1, Frame* curr);

  // Discards errored observations. Return true if
  // no observations were discarded.
  bool Clean(double error_threshold);

  // Normalize back to constant scale.
  void Normalize();

  // Show map stats on stdout
  void Stats() const;

  TrackedPoint* AddPoint(int id, const Vector4d& point);

  vector<std::unique_ptr<Camera>> cameras;
  vector<std::unique_ptr<Frame>> frames;
  vector<std::unique_ptr<TrackedPoint>> points;
};

#endif /* LOCALMAP_H_ */
