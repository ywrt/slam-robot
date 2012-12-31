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

#include "slam.h"


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
class HomogenousParameterization : public ceres::LocalParameterization {
 public:
  virtual ~HomogenousParameterization() {}

  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
  bool Plus(const double* x,
           const double* delta,
           double* x_plus_delta) const {
    Map<const Vector4d> mx(x);
    Map<const Vector3d> mdelta(delta);
    Map<Vector4d> mx_plus_delta(x_plus_delta);

    Vector4d sum = mx;
    sum.topLeftCorner<3,1>() += mdelta;
    mx_plus_delta = sum / sum.norm();
    return true;
  }
  bool ComputeJacobian(
      const double* x,
      double* jacobian) const {
    double ssq = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
    double ssq32 = sqrt(ssq)*ssq;

    jacobian[0]  = -x[0]*x[0]/ssq32 + 1/sqrt(ssq);
    jacobian[3]  = -x[0]*x[1]/ssq32;
    jacobian[6]  = -x[0]*x[2]/ssq32;
    jacobian[9]  = -x[0]*x[3]/ssq32;

    jacobian[1]  = -x[1]*x[0]/ssq32;
    jacobian[4]  = -x[1]*x[1]/ssq32 + 1/sqrt(ssq);
    jacobian[7]  = -x[1]*x[2]/ssq32;
    jacobian[10] = -x[1]*x[3]/ssq32;

    jacobian[2]  = -x[2]*x[0]/ssq32;
    jacobian[5]  = -x[2]*x[1]/ssq32;
    jacobian[8]  = -x[2]*x[2]/ssq32 + 1/sqrt(ssq);
    jacobian[11] = -x[2]*x[3]/ssq32;
    return true;
  }
};

struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y)
  : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const scale,
      const T* const intrinsics,
      const T* const camera_rotation,
      const T* const camera_translation,
      const T* const point,
      T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
#if 0
    T p[3];
    ceres::QuaternionRotatePoint(camera_rotation, point, p);
#else
    Map<const Quaternion<T> > q(camera_rotation);
    Map<const Matrix<T, 4, 1> > mpoint(point);

    Matrix<T, 3, 1> p;
    p(0) = mpoint(0);
    p(1) = mpoint(1);
    p(2) = mpoint(2);
    p = q * p;
#endif
    // camera[3,4,5] are the translation.
    p[0] += camera_translation[0] * point[3];
    p[1] += camera_translation[1] * point[3];
    p[2] += camera_translation[2] * point[3];
    // Compute the center of distortion.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    // Apply second and fourth order radial distortion.
    const T& l1 = intrinsics[0];
    const T& l2 = intrinsics[1];
    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2);

    // Compute final projected point position.
    const T& focal = scale[0];
    T predicted_x = distortion * xp;
    T predicted_y = focal * distortion * yp;
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    return true;
  }
  double observed_x;
  double observed_y;
};


void Slam::SetupParameterization() {
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;
  for (auto frame : frame_set_) {
    problem_->SetParameterization(frame->rotation(),
                                 quaternion_parameterization);
  }

#if 0
  auto homogenous = new HomogenousParameterization;
  for (auto& pt : point_set) {
    problem_.SetParameterization(pt->location(), homogenous);
  }
#endif
}

void Slam::SetupConstantBlocks(const int frame,
                         int min_frame_to_solve,
                         LocalMap* map) {
  problem_->SetParameterBlockConstant(map->frames[0].translation());
  problem_->SetParameterBlockConstant(map->frames[0].rotation());
  if (map->camera.data[0] < 0.3)
    map->camera.data[0] = 1;

  if (map->camera.data[0] > 3)
    map->camera.data[0] = 1;

  const bool fixed_intrinsics = true;
  if (frame < 95 || min_frame_to_solve > 0 || fixed_intrinsics)
    problem_->SetParameterBlockConstant(&(map->camera.data[0]));

  if (frame < 95 || min_frame_to_solve > 0 || fixed_intrinsics)
    problem_->SetParameterBlockConstant(&(map->camera.data[1]));

  for (int i = 1; i < (int) ((map->frames.size())) && i < min_frame_to_solve;
      ++i) {
    auto f = &(map->frames[i]);
    if (!frame_set_.count(f))
      continue;

    problem_->SetParameterBlockConstant(f->translation());
    problem_->SetParameterBlockConstant(f->rotation());
  }
  if (min_frame_to_solve >= 0) {
    for (const auto& point : point_set_) {
      if (point->last_frame() >= min_frame_to_solve)
        continue;

      problem_->SetParameterBlockConstant(point->location());
    }
  }
}

bool Slam::SetupProblem(int min_frame_to_solve,
                        LocalMap* map) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  problem_.reset(new ceres::Problem);
  frame_set_.clear();
  point_set_.clear();

  /*
   * 0.01 = focal 0.860662 r1 -0.061787 r2 0.000518
Iterations: 9078, error 0.018367

real  1m33.450s
user  2m1.368s
sys 0m2.366s
   *
   *0.015 = focal 0.946058 r1 -0.047019 r2 -0.009049
Iterations: 6844, error 0.018339

real  0m34.897s
user  0m49.609s
sys 0m0.655s
   *
   * 0.02 = focal 1.005811 r1 -0.056277 r2 0.000253
Iterations: 7185, error 0.019540

real  0m28.428s
user  0m41.957s
sys 0m0.620s
   * and lost track.
   */

  auto loss = new ceres::CauchyLoss(.015);
  //auto loss = new ceres::HuberLoss(0.02);
  for (auto& point : map->points) {
    if (point.observations_.size() < 2)
      continue;
    for (const auto& o : point.observations_) {
      CHECK_GE(o.frame_ref, 0);
      CHECK_LT(o.frame_ref, map->frames.size());

      // Each residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionError, 2, 1, 2, 4, 3, 4>(
              new ReprojectionError(o.pt(0), o.pt(1)));

      problem_->AddResidualBlock(cost_function,
                                loss /* squared loss */,
                                &(map->camera.data[0]),
                                &(map->camera.data[1]),
                                map->frames[o.frame_ref].rotation(),
                                map->frames[o.frame_ref].translation(),
                                point.location());
      Frame* f = &(map->frames[o.frame_ref]);
      frame_set_.insert(f);
    }
    point_set_.insert(&point);
  }

  if (frame_set_.size() < 2)
    return false;

  return true;
}

void Slam::Run(LocalMap* map, int min_frame_to_solve) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.

  ceres::Solver::Options options;
  if (!SetupProblem(min_frame_to_solve, map))
    return;

  const int frame = map->frames.size() - 1;

  ceres::ParameterBlockOrdering* ordering =
      new ceres::ParameterBlockOrdering;
  for (auto p : point_set_) {
    ordering->AddElementToGroup(p->location(), 0);
  }
  for (auto frame : frame_set_) {
    ordering->AddElementToGroup(frame->translation(), 1);
    ordering->AddElementToGroup(frame->rotation(), 2);
  }
  ordering->AddElementToGroup(&(map->camera.data[0]), 3);
  ordering->AddElementToGroup(&(map->camera.data[1]), 4);

  options.linear_solver_ordering = ordering;

  SetupParameterization();
  SetupConstantBlocks(frame, min_frame_to_solve, map);

  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.minimizer_progress_to_stdout = true;
  //options.use_inner_iterations = true;
  options.max_num_iterations = 150;
  if (min_frame_to_solve < 0)
    options.max_num_iterations = 1500;
  options.function_tolerance = 1e-6;
  //  if (frames > 15) {
  //  options.use_nonmonotonic_steps = true;
  //  }
  options.use_block_amd = true;
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
    point.location_.normalize();
    for (auto& o : point.observations_) {
      o.error = o.pt;
      const Frame& frame = map->frames[o.frame_ref];

      ReprojectionError project(o.pt(0), o.pt(1));

      project(map->camera.scale(),
              map->camera.instrinsics(),
              frame.rotation(),
              frame.translation(),
              point.location(),
              o.error.data());
    }
  }
}

