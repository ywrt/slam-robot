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

// Parameterization of homogeneous coordinates in R^3.
// An implementation of x + delta(x) that preserves the
// homogenization.
struct HomogenousPlus {
  template<typename T>
  bool operator()(const T* x, const T* delta, T* delta_plus) {
    Map<const Matrix<T, 4, 1>> mx(x);
    Map<const Matrix<T, 3, 1>> mdelta(delta);
    Map<Matrix<T, 4, 1>> mdelta_plus(delta_plus);

    // TODO: work out why topLeftCorner<3, 1>() doesn't work.
    mdelta_plus.topLeftCorner(3, 1) = mx.topLeftCorner(3, 1) * mx(3, 0);
    mdelta_plus(3,0) = mx(3,0);
    mdelta_plus.normalize();
    
    return true;
  }
};

struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y) :
      observed_x(observed_x), observed_y(observed_y) { }

  template <typename T>
  bool operator()(
      const T* const xyscale,
      const T* const distortion,

      const T* const frame_rotation,  // eigen quarternion: [x,y,z,w]
      const T* const frame_translation,  // [x,y,z]

      const T* const point,  // homogenous coordinates. [x,y,z,w]
      T* residuals) const {

    T projected[2];

    if (!project(xyscale, distortion, frame_rotation, frame_translation, point, projected))
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
  for (auto frame : pose_set_) {
    problem_->SetParameterization(frame->rotation(),
                                 quaternion_parameterization);
  }

#if 0
  auto homogenous = new AutoDiffLocalParameterization<HomogenousPlus, 3, 3>;
  for (auto& pt : point_set) {
    problem_.SetParameterization(pt->location(), homogenous);
  }
#endif
}

void Slam::SetupConstantBlocks(const int frame,
                         int min_frame_to_solve,
                         bool solve_cameras,
                         LocalMap* map) {
 // problem_->SetParameterBlockConstant(map->frames[0].translation());
 // problem_->SetParameterBlockConstant(map->frames[0].rotation());

  if (!solve_cameras) {
    for (auto& camera : map->cameras) {
      problem_->SetParameterBlockConstant(camera->xyscale());
      problem_->SetParameterBlockConstant(camera->distortion());
    }
  }

  for (int i = 1; i < (int) ((map->frames.size())) && i < min_frame_to_solve;
      ++i) {
    auto f = &(map->frames[i]->pose);
    if (!pose_set_.count(f))
      continue;

    problem_->SetParameterBlockConstant(f->translation());
    problem_->SetParameterBlockConstant(f->rotation());
  }

  if (min_frame_to_solve >= 0) {
    for (const auto& point : point_set_) {
      if (point->last_frame() >= min_frame_to_solve)
        continue;

   //   problem_->SetParameterBlockConstant(point->location());
    }
  }
}

bool Slam::SetupProblem(int min_frame_to_solve,
                        LocalMap* map) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  problem_.reset(new ceres::Problem);
  pose_set_.clear();
  point_set_.clear();

  auto loss = new ceres::CauchyLoss(.01);
  //auto loss = new ceres::HuberLoss(0.02);
  for (auto& point : map->points) {
    if (point->observations_.size() < 2)
      continue;
    for (const auto& o : point->observations_) {
      CHECK_GE(o.frame_idx, 0);
      CHECK_LT(o.frame_idx, map->frames.size());

      // Each residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionError, 2, 2, 2, 4, 3, 4>(
              new ReprojectionError(o.pt(0), o.pt(1)));

      const auto& frame = map->frames[o.frame_idx];
      problem_->AddResidualBlock(cost_function,
                                loss /* squared loss */,
                                frame->camera->xyscale(),
                                frame->camera->distortion(),

                                frame->pose.rotation(),
                                frame->pose.translation(),

                                point->location().data());

      camera_set_.insert(frame->camera);

      Pose* f = &(map->frames[o.frame_idx]->pose);
      pose_set_.insert(f);
    }
    point_set_.insert(point.get());
  }

  if (pose_set_.size() < 2)
    return false;

  return true;
}

void Slam::Run(LocalMap* map,
               int min_frame_to_solve,
               bool solve_cameras) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.

  ceres::Solver::Options options;
  if (!SetupProblem(min_frame_to_solve, map))
    return;

  const int frame = map->frames.size() - 1;
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
  SetupConstantBlocks(frame, min_frame_to_solve,
                      solve_cameras, map);

  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.minimizer_progress_to_stdout = true;
  //options.use_inner_iterations = true;
  options.max_num_iterations = 150;
  if (min_frame_to_solve < 0)
    options.max_num_iterations = 1500;
  options.function_tolerance = 1e-8;
  if (solve_cameras)
    options.function_tolerance = 1e-9;
  //  if (frames > 15) {
  //  options.use_nonmonotonic_steps = true;
  //  }
  //options.use_block_amd = true;
  options.num_threads = 3;
  //options.parameter_tolerance = 1e-9;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);
  if (min_frame_to_solve < 0)
    std::cout << summary.FullReport() << "\n";

  iterations_ += summary.iterations.size();
  error_ = summary.final_cost;
}

void Slam::ReprojectMap(LocalMap* map) {
  // Update observation error for each point.
  for (auto& point : map->points) {
    point->location_.normalize();
    for (auto& o : point->observations_) {
      o.error = o.pt;
      const auto& frame = map->frames[o.frame_idx];

      ReprojectionError project(o.pt(0), o.pt(1));

      project(
              frame->camera->xyscale(),
              frame->camera->distortion(),
              frame->pose.rotation(),
              frame->pose.translation(),
              point->location().data(),
              o.error.data());
    }
  }
}

