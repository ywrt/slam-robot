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

#include <eigen3/Eigen/Eigen>

using namespace Eigen;
using namespace std;

class TrackedPoint;

// Position and orientation of a frame camera.
// measures in world coordinates.
struct Pose {
  Pose() :
    rotation_ {1,0,0,0},
    translation_ {0,0,10} {}

  double* rotation() { return rotation_.coeffs().data(); }
  double* translation() { return translation_.data(); }
  const double* rotation() const { return rotation_.coeffs().data(); }
  const double* translation() const { return translation_.data(); }

  Quaterniond rotation_;
  Vector3d translation_;
};

// Camera intrinsics.
struct Camera {
  Camera() : scale_{1, 0.5626}, distortion_ {-0.01, 0} {}

  double* xyscale() { return &scale_[0]; }
  const double* xyscale() const { return &scale_[0]; }

  double* distortion() { return &distortion_[0]; }
  const double* distortion() const { return &distortion_[0]; }

  // 0 == focal length.
  // 1, 2 == radial distortion for r^2 and r^4
  double focal_;  // focal length.
  double scale_[2];  // x, y scale.
  double distortion_[2];  //  r^2 distortion, r^4 distortion.
};

// A frame taken by a specific camera in a specific pose.
struct Frame {
  Frame(int num, Camera* cam) : frame_num(num), camera(cam) { }

  // Project a tracked point into frame pixel space.
  bool Project(const Vector4d& point, Vector2d* result) const;

  // Convert a point in frame pixel space [-1,1]x[-1,1] and a
  // guess at distance into a homogenous point. 
  Vector4d Unproject(const Vector2d& point, double distance) const;

  int frame_num;
  Pose pose;
  Camera* camera;
};


// An observation of a tracked point from a specific frame.
struct Observation {
  Observation() : frame_idx(-1) {}
  Observation(Vector2d p, int frame_index) :
      pt(p), frame_idx(frame_index) { }

  Vector2d pt;  // In [-1, 1] x [-1, 1]
  Vector2d error;  // For debugging: filled in by slam.
  int frame_idx;
};

// A fully tracked point.
// Has the location in world-space, the descriptors, and any
// observations.
struct TrackedPoint {
  TrackedPoint() :
    location_ { 0, 0, 0, 1},
    bad_(0)
    { }

    // Pointer to an array of 4 doubles being [X, Y, Z, W],
    // the homogeous coordinates in world space.
    const Vector4d& location() const { return location_; }
    Vector4d& location() { return location_; }
 
    const Observation& last_obs() const { return observations_.back(); }   

    // The last frame in which this point was observed.
    int last_frame() const {
      if (observations_.size() < 1)
        return -1;
      return observations_.back().frame_idx;
    }

    // The most recent observation.
    const Vector2d& last_point() const {
      return observations_.back().pt;
    }

    // The number of observations of this point.
    int num_observations() const { return observations_.size(); }

    void AddObservation(const Observation& obs) { observations_.push_back(obs); }

    Vector4d location_;  // Homogeneous location in world.
    vector<Observation> observations_;
    int bad_;
    int id_;
};

// Description of the known world.
struct LocalMap {
  // Add a new (empty) frame to the map.
  // Returns the frame number.
  Frame* AddFrame(Camera* cam);

  // Add a new camera to the map
  Camera* AddCamera();

  Frame* frame(int frame_idx) { return frames[frame_idx].get(); }
  const Frame* frame(int frame_idx) const { return frames[frame_idx].get(); }

  // Do simple motion estimation from the previous two frames.
  void EstimateMotion(const Frame* f2, const Frame* f1, Frame* curr);

  // Discards errored observations.
  bool Clean();

  // Normalize back to constant scale.
  void Normalize();

  // Show map stats on stdout
  void Stats();

  TrackedPoint* AddPoint(int id, const Vector4d& point);

  vector<std::unique_ptr<Camera>> cameras;
  vector<std::unique_ptr<Frame>> frames;
  vector<std::unique_ptr<TrackedPoint>> points;
};

#endif /* LOCALMAP_H_ */
