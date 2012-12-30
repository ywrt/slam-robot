/*
 * slam.h
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

#ifndef SLAM_H_
#define SLAM_H_
#include <vector>

#include <Eigen/Eigen>

#include "octaveset.h"

using namespace Eigen;
using namespace std;

// Position and orientation of a frame camera.
struct Frame {
  Frame() :
    rotation_ {1,0,0,0},
    translation_ {0,0,10} {}

    double* rotation() { return rotation_.coeffs().data(); }
    double* translation() { return translation_.data(); }
    const double* rotation() const { return rotation_.coeffs().data(); }
    const double* translation() const { return translation_.data(); }

    Quaterniond rotation_;
    Vector3d translation_;
    // 0,1,2,3 == Quaternion
    // 4,5,6 == Camera translation
};

// Camera intrinsics.
struct Camera {
  Camera() : data {0.7, -0.05, 0} {}
  double data[3];
  // 0 == focal length.
  // 1, 2 == radial distortion
  double* scale() { return &data[0]; }
  const double* scale() const { return &data[0]; }
  double* instrinsics() { return &data[1]; }
  const double* instrinsics() const { return &data[1]; }
};

struct Observation {
  Vector2d pt;
  int frame_ref;
};

struct Descriptor {
  Descriptor(const uint8_t* v) {
    uint32_t* vv = (uint32_t*) v;
    for (int i = 0; i < 16; ++i) {
      data[i] = vv[i];
    }
  }
  int distance(const uint8_t* v) const {
    uint32_t* vv = (uint32_t*) v;
    int bits = 0;
    for (int i = 0; i < 16; ++i) {
      uint32_t d = data[i] ^ vv[i];
      int count = __builtin_popcount(d);
      bits += count;
    }
    return bits;
  }
  int distance(const Descriptor& desc) const {
    return distance((uint8_t*)desc.data);
  }

  uint32_t data[16];
};


// A fully tracked point.
// Has the location in world-space, the descriptors, and any
// observations.
struct TrackedPoint {
  TrackedPoint() :
    location_ { 0, 0, 0, 1},
    bad_(false)
    { }

    const double* location() const { return location_.data(); }
    double* location() { return location_.data(); }

    int last_frame() const {
      if (observations_.size() < 1)
        return -1;
      return observations_.back().frame_ref;
    }
    const Vector2d& last_point() const {
      return observations_.back().pt;
    }

    int num_observations() const { return observations_.size(); }

    // Match a descriptor against this point. 'max_distance'
    // is the maximum descriptor distance allowed to succeed.
    // If there's a match, return true and update 'max_distance'
    // to the found distance.
    bool match(const Descriptor& desc, int* max_distance) {
      bool matched = false;
      for (const auto& d : descriptors_) {
        int distance = d.distance(desc);
        if (distance > *max_distance)
          continue;
        *max_distance = distance;
        matched = true;
      }
      return matched;
    }

    Vector4d location_;  // Homogeneous location in world.
    vector<Observation> observations_;
    vector<Descriptor> descriptors_;
    vector<Patch> patches_;
    bool bad_;
};

// Description of the known world.
struct LocalMap {
  void addFrame(int frame_num) {
    CHECK_EQ(frames.size(), frame_num);

    if (frames.size() > 3) {
      auto s = frames.size();
      auto& f1 = frames[s - 1];
      auto& f2 = frames[s - 2];
      Vector3d motion = f1.translation_ - f2.translation_;
      if (motion.norm() > 1)
        motion /= motion.norm();

      Frame f = f1;
      f.translation_ += motion;
      frames.push_back(f);
    } else if (frames.size() > 0) {
      frames.push_back(frames.back());
      frames.back().translation()[0] += 0.01;
    } else {
      frames.push_back(Frame());
    }

  }

  Camera camera;
  vector<Frame> frames;
  vector<int> keyframes;
  vector<TrackedPoint> points;
  vector<Observation> obs;

};


class Slam {
};

#endif /* SLAM_H_ */
