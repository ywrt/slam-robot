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
  Camera() : center{0,0}, focal{1,1}, k1{0}, k2{0}, p1{0},p2{0},k3{0} {}

  // Takes frame coordinates and maps to (distorted) pixel coordinates.
  Vector2d Distort(const Vector2d& px) const {
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
  Vector2d Undistort(const Vector2d& px) const {
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

  Vector2d center;
  Vector2d focal;

  double k1, k2;
  double p1, p2;
  double k3;
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

  // Discards errored observations. Return true if
  // no observations were discarded.
  bool Clean(double error_threshold);

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
