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

class Frame;
class TrackedPoint;
class Observation;

// Camera intrinsics.
struct Camera {
  double kinit[7];  // k1, k2, k3, fx, fy, cx, cy
  double k[7];  // k1, k2, k3, fx, fy, cx, cy

  void Reset() {
    for (int i = 0; i < 7; ++i) {
      k[i] = kinit[i];
    }
  }

  // Map from [-1,1] x [-1,1] projective plane space 
  // to [0,width]x[0,height] pixel space.
  Vector2d PlaneToPixel(const Vector2d& p) const {
    double xp = p[0];
    double yp = p[1];

    double r2 = xp*xp + yp*yp;
    double distort = 1.0 + r2 * (k[0] + r2 * (k[1] + r2 * k[2]));

    xp *= distort;
    yp *= distort;

    xp *= k[3];
    yp *= k[4];

    xp += k[5];
    yp += k[6];
    return Vector2d(xp, yp);
  }

  Vector2d PixelToPlane(const Vector2d& p) const {
    double xp = p[0];
    double yp = p[1];
    
    xp -= k[5];
    yp -= k[6];

    xp /= k[3];
    yp /= k[4];

    // Estimate distortion iteratively.
    double x0 = xp;
    double y0 = yp;
    for (int i = 0; i < 3; ++i) {
      double r2 = xp*xp + yp*yp;
      double distort = 1./(1.0 + r2 * (k[0] + r2 * (k[1] + r2 * k[2])));

      xp = x0 * distort;
      yp = y0 * distort;
    }

    return Vector2d(xp, yp);
  }

};

// An observation of a tracked point from a specific frame.
struct Observation {
//  Observation() : frame(nullptr), is_disabled(0) {}
  Observation(Vector2d p, Frame* frame_ptr, TrackedPoint* tp) :
      pt(p), frame(frame_ptr), point(tp), is_disabled(0) { }

  void enable() { is_disabled = 0; }
  void disable() { is_disabled = 1; }
  bool enabled() const { return !is_disabled; }
  bool disabled() const { return is_disabled; }

  Vector2d pt;  // In pixel coordinates relative to frame.
  Frame* frame;
  TrackedPoint* point;
  int is_disabled;

  Vector2d error;  // For debugging: filled in by slam.
};

// A frame taken by a specific camera in a specific pose.
class Frame {
 public:
  Frame(int id, Camera* cam) :
    frame_id_(id),
    camera_(cam),
    rotation_{1, 0, 0, 0},
    translation_{0, 0, 0},
    previous_(nullptr) { }

  // Project a tracked point into frame pixel space.
  bool Project(const Vector4d& point, Vector2d* result) const;

  // Convert a point in frame pixel space [-1,1]x[-1,1] and a
  // guess at distance into a homogenous point. 
  Vector4d Unproject(const Vector2d& point, double distance) const;

  // Frame origin in world space.
  Vector3d position() const {
    return translation_;
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

  // Add a new observation of this point.
  void AddObservation(
      const Vector2d& p,
      TrackedPoint* tp) {
    observations_.push_back(std::unique_ptr<Observation>(
          new Observation(p, this, tp)));
  }

  const vector<std::unique_ptr<Observation>>& observations() const { return observations_; }

  // This frame is being kept. Add the observations to the
  // cache in TrackedPoint.
  void Commit();

  void set_previous(Frame* f) { previous_ = f; }
  Frame* previous() const { return previous_; }

  bool is_keyframe_;
 private:
  int frame_id_;
  Camera* camera_;
  Quaterniond rotation_;
  Vector3d translation_;
  Frame* previous_;

  vector<std::unique_ptr<Observation>> observations_;
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
    uncertainty_(1e8),
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
  const vector<Observation*>& observations() const { return observations_; }
  //vector<Observation*>& observations() { return observations_; }

  // The number of observations of this point.
  int num_observations() const { return observations_.size(); }
  // Return an observation. If idx is negative, then observations relative to the
  // most recent. -1 == most recent, -2 == previous most recent, etc.
  const Observation& observation(int idx) const {
    if (idx >= 0) {
      return *observations_[idx];
    } else {
      return *observations_[observations_.size() + idx];
    }
  }   
  Observation* observation(int idx) {
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

  double uncertainty() const { return uncertainty_; }
  void set_uncertainty(double u) { uncertainty_ = u; }

  // Add a new observation of this point.
  void AddObservation(Observation* obs);

  // Remove the last observation, checking that it's
  // from frame 'f'.
  bool RemoveObservation(Frame* f) {
    if (!observations_.size()) return false;
    if (observations_.back()->frame != f) return false;
    observations_.pop_back();
    CheckFlags();
    return true;
  }

  // Clear incorrectly set flags.
  void CheckFlags();

 private:
  Vector4d location_;  // Homogeneous location in world.
  double uncertainty_;
  int id_;
  int flags_;

  // List of observations of this point. This is a cache: These
  // pointers are owned in Frame, not here.
  vector<Observation*> observations_;
};

//
// Description of the known world.
// 
struct LocalMap {
  // Add a new (empty) frame to the map.
  // Returns the frame number.
  Frame* AddFrame(Camera* cam);

  // Remove and destroy the most recently added frame.
  void PopFrame();
  void CheckNotMoving();

  // Add a new camera to the map
  void AddCamera(Camera* cam);

  Frame* frame(int frame_idx) { return frames[frame_idx].get(); }
  const Frame* frame(int frame_idx) const { return frames[frame_idx].get(); }

  // Do simple motion estimation from the previous two frames.
  void EstimateMotion(const Frame* f2, const Frame* f1, Frame* curr);

  // Discards errored observations. Return true if
  // no observations were discarded.
  bool Clean(double error_threshold);


  void ApplyEpipolarConstraint();

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
