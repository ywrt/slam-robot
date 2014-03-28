/*
 * slam.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/ordered_groups.h>

#include <map>

#include "project.h"

#include "slam.h"

Slam::Slam() : iterations_(0), error_(0) {}
Slam::~Slam() { }

Vector2d fposToVector(const FPos& fp) {
  Vector2d r;
  r << fp.x * 2 - 1, fp.y * 2 - 1;
  return r;
}

FPos vectorToFPos(const Vector2d& v) {
  FPos fp((v(0) +1 ) / 2, (v(1) + 1) / 2);
  return fp;
}

// Eigen compatible quaternion parameterization
class QuaternionParameterization : public ceres::LocalParameterization {
public:
  virtual ~QuaternionParameterization() {}
  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
  virtual bool Plus(
      const double* x,
      const double* delta,
      double* x_plus_delta) const {
    Map<const Quaterniond> mx(x);
    Map<const Vector3d> mdelta(delta);
    Map<Quaterniond> mx_plus_delta(x_plus_delta);
    const double norm_delta = mdelta.norm();
    if (norm_delta > 0.0) {
      const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);

      Quaterniond q_delta;
      q_delta.vec() = sin_delta_by_delta * mdelta;
      q_delta.w() = cos(norm_delta);

      mx_plus_delta = q_delta * mx;
    } else {
      mx_plus_delta = mx;
    }
    return true;
  }
  virtual bool ComputeJacobian(const double* x, double* jacobian) const {
    jacobian[0] = -x[0]; jacobian[1]  = -x[1]; jacobian[2]  = -x[2];  // NOLINT
    jacobian[3] =  x[3]; jacobian[4]  =  x[2]; jacobian[5]  = -x[1];  // NOLINT
    jacobian[6] = -x[2]; jacobian[7]  =  x[3]; jacobian[8]  =  x[0];  // NOLINT
    jacobian[9] =  x[1]; jacobian[10] = -x[0]; jacobian[11] =  x[3];  // NOLINT
    return true;
  }
};

struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y) :
      observed_x(observed_x), observed_y(observed_y) { }

  template <typename T>
  bool operator()(
      const T* const frame_rotation,  // eigen quarternion: [x,y,z,w]
      const T* const frame_translation,  // [x,y,z]

      const T* const point,  // homogenous coordinates. [x,y,z,w]
      T* residuals) const {

    T projected[2];

    if (!project(frame_rotation, frame_translation, point, projected))
      return false;  // Point is behind camera.

    // The error is the difference between the predicted and observed position.
    residuals[0] = projected[0] - T(observed_x);
    residuals[1] = projected[1] - T(observed_y);
    return true;
  }

  ProjectPoint project;
  double observed_x;
  double observed_y;
};


void Slam::SetupParameterization() {
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;
  for (auto frame : frame_set_) {
    problem_->SetParameterization(frame->pose.rotation(),
                                 quaternion_parameterization);
  }

#if 0
  auto homogenous = new AutoDiffLocalParameterization<HomogenousPlus, 3, 3>;
  for (auto& pt : point_set) {
    problem_.SetParameterization(pt->location(), homogenous);
  }
#endif
}

void Slam::SetupConstantBlocks(
    LocalMap* map,
    std::function<bool (int frame_idx)> solve_frame_p) {

  // problem_->SetParameterBlockConstant(map->frames[0].translation());
  // problem_->SetParameterBlockConstant(map->frames[0].rotation());

  if (!solve_frame_p)
    return;  // Solve all frames == no constant frames.

  cout << "Setting frames ";
  for (auto& frame : frame_set_) {
    if (solve_frame_p(frame->frame_num))
      continue;
    cout << frame->frame_num << ", ";
    problem_->SetParameterBlockConstant(frame->pose.translation());
    problem_->SetParameterBlockConstant(frame->pose.rotation());
  }
  cout << " to const.\n";
}

bool Slam::SetupProblem(
    LocalMap* map,
    std::function<bool (int frame_idx)> solve_frame_p) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for points are added automatically.
  problem_.reset(new ceres::Problem);
  frame_set_.clear();
  point_set_.clear();

  auto loss = new ceres::CauchyLoss(.005);
  //auto loss = new ceres::HuberLoss(0.02);

  // Search for the list of frames to be using. We use the set of
  // all frames that reference points that are references by frames
  // that need solving. 
  // TODO: This is wasteful.
  std::set<int> frames_to_use;
  for (auto& point : map->points) {
    if (!point->slam_usable())
      continue;  // Not (yet?) usable for SLAM problem.

    bool use_point = false;
    for (const auto& o : point->observations()) {
      if (o.disabled()) {
        continue;
      }

      if (solve_frame_p && !solve_frame_p(o.frame->num())) {
        continue;  // Not using this frame for solving.
      }

      // We do want this point.
      use_point = true;
      break;
    }
    if (!use_point)
      continue;

    for (const auto& o : point->observations()) {
      if (o.disabled())
        continue;
      frames_to_use.insert(o.frame->num());
    }
    point_set_.insert(point.get());
  }

  // Now add the set of points, observations, and frame poses to the
  // SLAM problem.
  for (auto point : point_set_) {
    for (const auto& o : point->observations()) {
      if (o.disabled())
        continue;

      if (!frames_to_use.count(o.frame->num()))
        continue;

      CHECK_LT(o.frame->num(), map->frames.size());

      // Each residual block takes a point and frame pose as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 4>(
              new ReprojectionError(o.pt(0), o.pt(1)));

      problem_->AddResidualBlock(cost_function,
                                loss /* squared loss */,

                                o.frame->pose.rotation(),
                                o.frame->pose.translation(),

                                point->location().data());

      camera_set_.insert(o.frame->camera);

      frame_set_.insert(o.frame);
    }
  }

  if (frame_set_.size() < 2) {
    cout << "Slam aborted due to frame set too small. " << frame_set_.size() << " and point set " << point_set_.size() << "\n";
    return false;
  }

  return true;
}

void Slam::Run(LocalMap* map,
               std::function<bool (int frame_idx)> solve_frame_p) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.

  ceres::Solver::Options options;
  if (!SetupProblem(map,
        solve_frame_p
        ))
    return;

#if 0
  ceres::ParameterBlockOrdering* ordering =
      new ceres::ParameterBlockOrdering;
  for (auto p : point_set_) {
    ordering->AddElementToGroup(p->location(), 0);
  }
  for (auto frame : pose_set_) {
    ordering->AddElementToGroup(frame->translation(), 1);
    ordering->AddElementToGroup(frame->rotation(), 2);
  }
  ordering->AddElementToGroup(&(map->camera.data[0]), 3);
  ordering->AddElementToGroup(&(map->camera.data[1]), 4);
  options.linear_solver_ordering = ordering;
#endif

  SetupParameterization();
  SetupConstantBlocks(map, solve_frame_p);

  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.minimizer_progress_to_stdout = true;
  //options.use_inner_iterations = true;
  options.max_num_iterations = 100;
  if (!solve_frame_p)
    options.max_num_iterations = 100;
  options.function_tolerance = 1e-7;
  //  if (frames > 15) {
  //  options.use_nonmonotonic_steps = true;
  //  }
  //options.use_block_amd = true;
  options.num_threads = 3;
  //options.parameter_tolerance = 1e-9;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  if (!solve_frame_p)
    std::cout << summary.FullReport() << "\n";

  iterations_ += summary.iterations.size();
  error_ = summary.final_cost;
}

double Slam::ReprojectMap(LocalMap* map) {
  // Update observation error for each point.
  double mean(0);
  double count(0);
  for (auto& point : map->points) {
    for (auto& o : point->observations()) {
      o.error = o.pt;

      ReprojectionError project(o.pt(0), o.pt(1));

      bool result = project(
              o.frame->pose.rotation(),
              o.frame->pose.translation(),
              point->location().data(),
              o.error.data());
      if (!result)
        continue;  // Point can't be projected?
      mean = mean + (o.error.norm() - mean) / (count + 1);
      ++count;
    }
  }
  return mean;
}

