/*
 * slam.h
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

#ifndef SLAM_H_
#define SLAM_H_

#include <memory>
#include <set>

#include "localmap.h"
#include "imgtypes.h"

Vector2d fposToVector(const FPos& fp);
FPos vectorToFPos(const Vector2d& v);

namespace ceres {
class Problem;
}

class Slam {
 public:
  Slam();
  virtual ~Slam();

  void Run(LocalMap* map,
           int min_frame_to_solve,
           bool solve_camera);

  // TODO: This belong in LocalMap which means the projection
  // should be lifted out.
  void ReprojectMap(LocalMap* map);

  // Project a point into a frame.
  void Project(const Frame* frame, const TrackedPoint* point) const;

  int iterations() const { return iterations_; }
  double error() const { return error_; }

 private:
  unique_ptr<ceres::Problem> problem_;
  std::set<Pose*> pose_set_;
  std::set<TrackedPoint*> point_set_;
  std::set<Camera*> camera_set_;

  void SetupParameterization();
  void SetupConstantBlocks(const int frame,
                           int min_frame_to_solve,
                           bool solve_camera,
                           LocalMap* map);
  bool SetupProblem(int min_frame_to_solve, LocalMap* map);

  int iterations_;
  double error_;
};

#endif /* SLAM_H_ */
