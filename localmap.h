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
  Camera() : data {0.5625, -0.01, 0} {}

  double* scale() { return &data[0]; }
  const double* scale() const { return &data[0]; }
  double* instrinsics() { return &data[1]; }
  const double* instrinsics() const { return &data[1]; }

  // 0 == focal length.
  // 1, 2 == radial distortion for r^2 and r^4
  double data[3];
};

struct Observation {
  Observation() : frame_idx(-1) {}
  Observation(int x, int y, int frame_index, int camera_index) :
      pt({x, y}), frame_idx(frame_index), camera_idx(camera_index) {}

  Vector2d pt;
  Vector2d error;
  int frame_idx;
  int camera_idx;
};

// A fully tracked point.
// Has the location in world-space, the descriptors, and any
// observations.
struct TrackedPoint {
  TrackedPoint() :
    location_ { 0, 0, 0, 1},
    bad_(false)
    { }

    // Pointer to an array of 4 doubles being [X, Y, Z, W],
    // the homogeous coordinates in world space.
    const double* location() const { return location_.data(); }
    double* location() { return location_.data(); }

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

    Vector4d location_;  // Homogeneous location in world.
    vector<Observation> observations_;
    bool bad_;
};

// Description of the known world.
struct LocalMap {
  // Add a new (empty) frame to the map.
  // Returns the frame number.
  int AddFrame();

  // Discards errored observations.
  void Clean();

  TrackedPoint* AddPoint();

  Camera camera;
  vector<Pose> frames;
  vector<std::unique_ptr<TrackedPoint>> points;
};

#endif /* LOCALMAP_H_ */
