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
           std::function<bool (int frame_idx)> solve_frame_p
           );

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
  std::set<Camera*> camera_set_;

  void SetupParameterization();
  void SetupConstantBlocks(
      LocalMap* map,
      std::function<bool (int frame_idx)> solve_frame_p);
  bool SetupProblem(
      LocalMap* map,
      std::function<bool (int frame_idx)> solve_frame_p);

  int iterations_;
  double error_;
};

#endif /* SLAM_H_ */
