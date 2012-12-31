/*
 * slam.h
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

#ifndef SLAM_H_
#define SLAM_H_

#include <memory>

#include "localmap.h"

Vector2d fposToVector(const FPos& fp);
FPos vectorToFPos(const Vector2d& v);

namespace ceres {
class Problem;
}

class Slam {
 public:
  Slam() : iterations_(0), error_(0) {}

  void Run(LocalMap* map,
           int min_frame_to_solve,
           bool solve_camera);
  // TODO: This belong in LocalMap which means the projection
  // should be lifted out.
  void ReprojectMap(LocalMap* map);

  int iterations() const { return iterations_; }
  double error() const { return error_; }

 private:
  unique_ptr<ceres::Problem> problem_;
  set<Frame*> frame_set_;
  set<TrackedPoint*> point_set_;

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
