/*
 * slam.h
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

#ifndef SLAM_H_
#define SLAM_H_

#include <functional>
#include <memory>
#include <set>

#include "localmap.h"

namespace ceres {
class Problem;
}

class Slam {
 public:
  Slam();
  virtual ~Slam();

  bool SolveFrames(
      LocalMap* map,
      int num_to_solve,
      int num_to_present,
      double range);

  bool SolveAllFrames(
      LocalMap* map,
      double range,
      bool solve_cameras);

  bool SolveFramePose(
      const Frame* f1,
      Frame* f2);

  // TODO: This belong in LocalMap which means the projection
  // should be lifted out.
  double ReprojectMap(LocalMap* map);

  // Project a point into a frame.
  void Project(const Frame* frame, const TrackedPoint* point) const;

  int iterations() const { return iterations_; }
  double error() const { return error_; }

 private:
  unique_ptr<ceres::Problem> problem_;

  std::set<Frame*> frame_set_;
  std::set<TrackedPoint*> point_set_;

  bool SetupProblem(
      double range,
      const std::map<Frame*, bool>& frames);

  bool Run(bool fine);

  int iterations_;
  double error_;
};

#endif /* SLAM_H_ */
