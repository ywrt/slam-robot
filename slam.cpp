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
virtual bool Plus(const double* x,
    const double* delta,
    double* x_plus_delta) const;
virtual bool ComputeJacobian(const double* x,
    double* jacobian) const;
virtual int GlobalSize() const { return 4; }
virtual int LocalSize() const { return 3; }
};

bool HomogenousParameterization::Plus(const double* x,
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

bool HomogenousParameterization::ComputeJacobian(
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

void Project(
    const Camera& camera,
    const Frame& frame,
    const TrackedPoint& point,
    double* result) {
  ReprojectionError p(result[0],result[1]);
  p(camera.scale(), camera.instrinsics(),
      frame.rotation(),
      frame.translation(),
      point.location(),
      result);
}


void RunSlam(LocalMap* map, int min_frame_to_solve) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  ceres::Solver::Options options;

  const int frame = map->frames.size() - 1;

  set<Frame*> frame_set;
  set<TrackedPoint*> point_set;

  auto loss = new ceres::CauchyLoss(.02);
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

      problem.AddResidualBlock(cost_function,
          loss /* squared loss */ ,
          &(map->camera.data[0]),
          &(map->camera.data[1]),
          map->frames[o.frame_ref].rotation(),
          map->frames[o.frame_ref].translation(),
          point.location());
      Frame* f = &(map->frames[o.frame_ref]);
      frame_set.insert(f);
    }
    point_set.insert(&point);
  }

  if (frame_set.size() < 2)
    return;

#if 1
  options.use_inner_iterations = true;
#if 0
  options.inner_iteration_ordering =
      new ceres::ParameterBlockOrdering;
  for (auto& p : point_set) {
    options.inner_iteration_ordering->AddElementToGroup(
        p->location(), 0);
  }
#endif
#endif

#if 1
  ceres::ParameterBlockOrdering* ordering =
      new ceres::ParameterBlockOrdering;

  for (auto p : point_set) {
    ordering->AddElementToGroup(p->location(), 0);
  }
  for (auto frame : frame_set) {
    ordering->AddElementToGroup(frame->translation(), 1);
    ordering->AddElementToGroup(frame->rotation(), 2);
  }
  ordering->AddElementToGroup(&(map->camera.data[0]), 3);
  ordering->AddElementToGroup(&(map->camera.data[1]), 4);
  options.linear_solver_ordering = ordering;
#endif

  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;
  for (auto frame : frame_set) {
    problem.SetParameterization(frame->rotation(),
        quaternion_parameterization);
  }
  auto homogenous = new HomogenousParameterization;
  for (auto& pt : point_set) {
    problem.SetParameterization(pt->location(), homogenous);
  }

  problem.SetParameterBlockConstant(map->frames[0].translation());
  problem.SetParameterBlockConstant(map->frames[0].rotation());
  if (map->camera.data[0] < 0.3)
    map->camera.data[0] = 1;
  if (map->camera.data[0] > 3)
      map->camera.data[0] = 1;
  if (frame < 10)
    problem.SetParameterBlockConstant(&(map->camera.data[0]));
  if (frame < 50)
    problem.SetParameterBlockConstant(&(map->camera.data[1]));

  for (int i = 1; i < (int)map->frames.size() && i < min_frame_to_solve; ++i) {
    auto f = &(map->frames[i]);
    if (!frame_set.count(f))
      continue;
    problem.SetParameterBlockConstant(f->translation());
    problem.SetParameterBlockConstant(f->rotation());
  }

  if (min_frame_to_solve >= 0) {
    for (const auto& point : point_set) {
      if (point->last_frame() >= min_frame_to_solve)
        continue;
      problem.SetParameterBlockConstant(
          point->location());
    }
  }


  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.minimizer_progress_to_stdout = true;
  //options.use_inner_iterations = true;
  options.max_num_iterations = 150;
  if (min_frame_to_solve < 0)
    options.max_num_iterations = 1500;
  options.function_tolerance = 1e-8;
  //  if (frames > 15) {
  //  options.use_nonmonotonic_steps = true;
  //  }
  options.use_block_amd = true;
  options.num_threads = 3;
  options.parameter_tolerance = 1e-9;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}

