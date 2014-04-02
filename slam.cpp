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

struct FrameDistance {
  FrameDistance(double dist) : distance(dist) { }

  template <typename T>
  bool operator()(
      const T* const f1_rotation,  // eigen quarternion: [x,y,z,w]
      const T* const f1_translation,  // [x,y,z]
      const T* const f2_rotation,  // eigen quarternion: [x,y,z,w]
      const T* const f2_translation,  // [x,y,z]
      T* residual) const {

    Eigen::Map<const Eigen::Quaternion<T> > r1(f1_rotation);
    Eigen::Map<const Eigen::Quaternion<T> > r2(f2_rotation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > t1(f1_translation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > t2(f2_translation);

    T dist = (r1.matrix().inverse() * t1 - r2.matrix().inverse() * t2).norm();
    // The error is the difference between the predicted and observed position.
    residual[0] = 0.001 * (dist - T(distance));
    return true;
  }

  double distance;
};


void Slam::SetupParameterization() {
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;
  for (auto frame : frame_set_) {
    problem_->SetParameterization(frame->rotation().coeffs().data(),
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
    std::function<bool (Frame* frame_idx)> solve_frame_p) {

  if (!solve_frame_p)
    return;  // Solve all frames == no constant frames.

  cout << "Setting frames ";
  for (auto& frame : frame_set_) {
    if (solve_frame_p(frame))
      continue;
    cout << frame->id() << ", ";
    problem_->SetParameterBlockConstant(frame->translation().data());
    problem_->SetParameterBlockConstant(frame->rotation().coeffs().data());
  }
  cout << " to const.\n";
}

bool Slam::SetupProblem(
    LocalMap* map,
    std::function<bool (Frame* frame_idx)> solve_frame_p) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for points are added automatically.
  problem_.reset(new ceres::Problem);
  frame_set_.clear();
  point_set_.clear();

  auto loss = new ceres::CauchyLoss(.01);
  //auto loss = new ceres::HuberLoss(0.02);

  // Search for the list of frames to be using. We use the set of
  // all frames that reference points that are references by frames
  // that need solving. 
  // TODO: This is wasteful.
  std::set<Frame*> frames_to_use;
  for (auto& point : map->points) {
    if (!point->slam_usable())
      continue;  // Not (yet?) usable for SLAM problem.

    bool use_point = false;
    for (const auto& o : point->observations()) {
      if (o.disabled()) {
        continue;
      }

      if (solve_frame_p && !solve_frame_p(o.frame)) {
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
      frames_to_use.insert(o.frame);
    }
    point_set_.insert(point.get());
  }

  // Now add the set of points, observations, and frame poses to the
  // SLAM problem.
  for (auto point : point_set_) {
    for (const auto& o : point->observations()) {
      if (o.disabled())
        continue;

      if (!frames_to_use.count(o.frame))
        continue;

      // Each residual block takes a point and frame pose as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 4>(
              new ReprojectionError(o.pt(0), o.pt(1)));

      problem_->AddResidualBlock(cost_function,
                                loss /* squared loss */,
                                o.frame->rotation().coeffs().data(),
                                o.frame->translation().data(),
                                point->location().data());

      frame_set_.insert(o.frame);
    }
  }

  // Constraint the frame-to-frame distances.
  if (1 || !solve_frame_p) {
    auto frame_loss = new ceres::CauchyLoss(.3);
    for (unsigned int i = 0; i < map->frames.size() - 1; ++i) {
      auto f1 = map->frames[i].get();
      auto f2 = map->frames[i+1].get();
      if (!frame_set_.count(f1) || !frame_set_.count(f2))
        continue;

      // Consequtive frames in the problem.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<FrameDistance, 1, 4, 3, 4, 3>(
              new FrameDistance(150.));

      problem_->AddResidualBlock(cost_function,
                                frame_loss /* squared loss */,
                                f1->rotation().coeffs().data(),
                                f1->translation().data(),
                                f2->rotation().coeffs().data(),
                                f2->translation().data());

      
    }
  }

  if (frame_set_.size() < 2) {
    cout << "Slam aborted due to frame set too small. " << frame_set_.size() << " and point set " << point_set_.size() << "\n";
    return false;
  }

  return true;
}

bool Slam::Run(LocalMap* map,
               std::function<bool (Frame* frame_idx)> solve_frame_p) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.

  ceres::Solver::Options options;
  if (!SetupProblem(map,
        solve_frame_p
        ))
    return false;

  SetupParameterization();
  SetupConstantBlocks(map, solve_frame_p);

  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  //options.minimizer_progress_to_stdout = true;
  //options.use_inner_iterations = true;
  options.max_num_iterations = 100;
  if (!solve_frame_p)
    options.max_num_iterations = 100;
  options.function_tolerance = 1e-7;
  //  if (frames > 15) {
  //  options.use_nonmonotonic_steps = true;
  //  }
  //options.use_block_amd = true;
  //options.num_threads = 3;
  //options.parameter_tolerance = 1e-9;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  if (!solve_frame_p) {
    std::cout << summary.FullReport() << "\n";
  } else {
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Error: " << summary.error << "\n";
  }

  iterations_ += summary.iterations.size();
  error_ = summary.final_cost;

  return summary.error.size() == 0;
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
              o.frame->rotation().coeffs().data(),
              o.frame->translation().data(),
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

