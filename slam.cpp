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
#include <unordered_set>
#include <unordered_map>

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
  ReprojectionError(const Eigen::Vector2d& obs) : observation(obs) {}

  template <typename T>
  bool operator()(
      const T* const frame_rotation,  // eigen quarternion: [x,y,z,w]
      const T* const frame_translation,  // [x,y,z]
      const T* const intrinsics,  // [k1, k2]
      const T* const point,  // homogenous coordinates. [x,y,z,w]
      T* residuals) const {

    T projected[2];

    if (!project(frame_rotation, frame_translation, intrinsics, point, projected))
      return false;  // Point is behind camera.

    // The error is the difference between the predicted and observed position.
    residuals[0] = projected[0] - T(observation(0));
    residuals[1] = projected[1] - T(observation(1));
    return true;
  }

  ProjectPoint project;
  Eigen::Vector2d observation;
};

struct FrameDistance {
  FrameDistance(double dist) : distance(dist) { }

  template <typename T>
  bool operator()(
      const T* const f1_translation,  // [x,y,z]
      const T* const f2_translation,  // [x,y,z]
      T* residual) const {

    Eigen::Map<const Eigen::Matrix<T, 3, 1> > t1(f1_translation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > t2(f2_translation);

    T dist = (t1 - t2).norm();
    // The error is the difference between the predicted and observed position.
    residual[0] = T(0.1) * (dist - T(distance));
    return true;
  }

  double distance;
};

struct CameraStabilization {
  template <typename T>
  bool operator()(
      const T* const intrinsics,  // [k1, k2]
      T* residual) const {

    residual[0] = T(1000.0) * intrinsics[0] * intrinsics[0];
    residual[1] = T(1000.0) * intrinsics[1] * intrinsics[1];
    residual[2] = T(1000.0) * intrinsics[2] * intrinsics[2];
    residual[3] = T(.1) * (intrinsics[3] - T(416.)) * (intrinsics[3] - T(416.));
    //residual[4] = T(.00001) * (intrinsics[4] - T(-416.)) * (intrinsics[4] - T(-416.));
    residual[4] = T(.1) * (intrinsics[4] + intrinsics[3]) * (intrinsics[4] + intrinsics[3]);
    residual[5] = T(.01) * (intrinsics[5] - T(320.)) * (intrinsics[5] - T(320.));
    residual[6] = T(.01) * (intrinsics[6] - T(240.)) * (intrinsics[6] - T(240.));

    return true;
  }
};

// Point observed from two frames. The epipolar constraint
// is that the obs1 * E * obs2 == 0.
struct EpipolarConstraint{
  EpipolarConstraint(const Eigen::Vector2d& pp1, const Eigen::Vector2d& pp2) : h1(pp1), h2(pp2) {}

  template <typename T>
  bool operator()(
      const T* const rotation,  // eigen quarternion: [x,y,z,w]
      const T* const translation,  // [x,y,z]
      T* residual) const {

    Eigen::Map<const Eigen::Quaternion<T> > r1(rotation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > t1(translation);

    Eigen::Matrix<T, 3, 3> skew;
    skew << T(0), -t1[2], t1[1],
            t1[2], T(0), -t1[0],
            -t1[1], t1[0], T(0);

    Eigen::Matrix<T, 3, 1> th1;
    Eigen::Matrix<T, 3, 1> th2;
    th1 << T(h1(0)), T(h1(1)), T(1);
    th2 << T(h2(0)), T(h2(1)), T(1);

    T result = th2.transpose() * skew * r1.matrix() * th1;

    residual[0] = result;
    return true;
  }

  Eigen::Vector2d h1;
  Eigen::Vector2d h2;
};

// Hacked up parameterization for the unit vector.
// takes vector and dx,dy and returns a new vector in r^3
class UnitVectorParameterization {
 public:
  template<typename T>
  bool operator()(const T* x, const T* delta, T* xplusd) const {
    Eigen::Map<Eigen::Matrix<T, 3, 1> > r(xplusd);

    r[0] = x[0] + delta[0];
    r[1] = x[1] + (-delta[0] - delta[1]);
    r[2] = x[2] + delta[1];
    r.normalize();
    return true;
  }
};

// Solve for frame position using only the epipolar constraint.
bool Slam::SolveFramePose(
    const Frame* f1,
    Frame* f2) {
  if (f1 == nullptr)
    return false;
  return false;

  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for points are added automatically.
  problem_.reset(new ceres::Problem);

  auto loss = new ceres::CauchyLoss(0.01);

  Eigen::Quaterniond rotation = f2->rotation() * f1->rotation().inverse();
  Eigen::Vector3d translation = (f1->translation() - f2->translation());
  double length = translation.norm();
  translation.normalize();

  // Build a map of point->observation for frame 1.
  std::unordered_map<TrackedPoint*, Observation*> f1_map(100);
  for (const auto& o : f1->observations())
    f1_map.insert(std::make_pair(o->point, o.get()));

  // For each point represented in both frames, add an
  // epipolar constraint.
  int count = 0;
  for (const auto& o : f2->observations()) {
    if (!f1_map.count(o->point))
      continue;  // Point isn't in the other frame.

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<EpipolarConstraint, 1, 4, 3>(
            new EpipolarConstraint(
              f1->camera()->PixelToPlane(f1_map[o->point]->pt),
              f2->camera()->PixelToPlane(o->pt)
            ));

    problem_->AddResidualBlock(cost_function,
                              loss /* squared loss */,
                              rotation.coeffs().data(),
                              translation.data());
    ++count;
  }


  if (count < 8) {
    cout << "Epipolar solve aborted due to count too small. " << count << "\n";
    return false;
  }

  // Add parameterization for the quarternion 
  problem_->SetParameterization(rotation.coeffs().data(),
      new ceres::QuaternionParameterization);

  // Add parameterization for the translation (which is assumed to be
  // unitary.
  problem_->SetParameterization(translation.data(),
      new ceres::AutoDiffLocalParameterization<UnitVectorParameterization, 3, 2>);


  // Run the non-linear solve.
  if (!Run(false)) {
    cout << "Epipolar solve failed\n";
    return false;
  }

  // We now know something about the relationship between f1 and f2.
  f2->rotation() = rotation * f1->rotation();
  f2->translation() = f1->translation() - translation * length;

  return true;
}

// Active frame = frame pose is being optimized for.
// Static frame = frame pose is fixed.
// Points with more than one optimization are always optimized for.
// Points observed from a set of frames where the maximum frame distance is less than X are singular
// and should be skipped.

// Solve some set of frame poses and point locations.
bool Slam::SetupProblem(
    double range,
    const std::map<Frame*, bool>& frames) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for points are added automatically.
  problem_.reset(new ceres::Problem);

  //auto loss = new ceres::HuberLoss(0.02);
  auto loss = new ceres::CauchyLoss(range);

  // Frames that are not referenced by any usable observation.
  std::unordered_set<Frame*> skip_frames(frames.size());
  // Points referenced by a presented frame.
  std::unordered_set<TrackedPoint*> point_set(100);
  // Points that reference a frame being solved for.
  std::unordered_set<TrackedPoint*> fluid_points(100);

  for (const auto& pair : frames) {
    Frame* frame = pair.first;
    bool is_const = pair.second;

    bool frame_used = false;
    for (const auto& o : frame->observations()) {
      if (o->disabled())
        continue;
      if (!o->point->slam_usable())
        continue;  // Not (yet?) usable for SLAM problem.

      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 7, 4>(
              new ReprojectionError(o->pt));

      problem_->AddResidualBlock(cost_function,
                                loss /* squared loss */,
                                o->frame->rotation().coeffs().data(),
                                o->frame->translation().data(),
                                o->frame->camera()->k,
                                o->point->location().data());
      frame_used = true;
      point_set.insert(o->point);
      if (!is_const)
        fluid_points.insert(o->point);
    }

    if (!frame_used)
      skip_frames.insert(frame);
  }

  if ((frames.size() - skip_frames.size()) < 2) {
    cout << "Slam aborted due to frame set too small. " << frames.size() << "\n";
    return false;
  }

  // Add parameterization for the frame quarternion and mark
  // frames that aren't being solved as const.
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres::QuaternionParameterization;

  std::set<int> const_set;
  std::set<int> id_set;
  for (const auto& pair : frames) {
    Frame* frame = pair.first;
    bool is_const = pair.second;

    if (skip_frames.count(frame))
      continue;

    problem_->SetParameterization(frame->rotation().coeffs().data(),
                                 quaternion_parameterization);

    id_set.insert(frame->id());
    if (is_const) {
      problem_->SetParameterBlockConstant(frame->translation().data());
      problem_->SetParameterBlockConstant(frame->rotation().coeffs().data());
      const_set.insert(frame->id());
    }
  }

  printf("Frames: ");
  printf("Const[ ");
  for (int id : const_set) { printf("%d, ", id); }
  printf(" ],  Solving[ ");
  for (int id : id_set) { if (!const_set.count(id)) printf("%d, ", id); }
  printf(" ]\n");
  const_set.clear();
  id_set.clear();

  // Mark points that are not referenced by a frame being solved for as const.
  for (const auto& point : point_set) {
    id_set.insert(point->id());
    if (point->uncertainty() > 100)
      continue;  // Not quite sure where the point is. Try harder.

    if (!fluid_points.count(point)) {
      problem_->SetParameterBlockConstant(point->location().data());
      const_set.insert(point->id());
      continue;
    }
#if 0
    // Mark points that reference unpresented frames as const.
    for (const auto& o : point->observations()) {
      if (o->disabled())
        continue;
      if (frames.count(o->frame))
        continue;  // is in the presented frame set.

      // Point references a frame that's not presented. Mark it const
      // as we can't safely solve it.
      //problem_->SetParameterBlockConstant(point->location().data());
      //const_set.insert(point->id());
      break;
    }
#endif
  }

  printf("Points: ");
  printf("Const[ ");
  for (int id : const_set) { printf("%d, ", id); }
  printf(" ],  Solving[ ");
  for (int id : id_set) { if (!const_set.count(id)) printf("%d, ", id); }
  printf(" ]\n");
  const_set.clear();
  id_set.clear();


  // Constraint the frame-to-frame distances.
  for (const auto& pair : frames) {
    Frame* frame = pair.first;
    bool is_const = pair.second;

    if (is_const)
      continue;  // Not being solved for.

    if (skip_frames.count(frame))
      continue;  // Not being solved for.

    Frame* prev = frame->previous();
    if (!frames.count(prev))
      continue;  // Previous frame isn't in presented set.


    // Weakly constrain the frame to be 150mm distant from the
    // previous frame. This is assuming a stero camera setup
    // with a 150mm baseline.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<FrameDistance, 1, 3, 3>(
            new FrameDistance(150.));
    auto frame_loss = new ceres::CauchyLoss(15);

    problem_->AddResidualBlock(
        cost_function,
        frame_loss /* squared loss */,
        frame->translation().data(),
        prev->translation().data());
  }

  return true;
}

// Solve a contiguous subset of frames.
bool Slam::SolveFrames(
      LocalMap* map,
      int num_to_solve,
      int num_to_present,
      double range) {

  std::map<Frame*, bool> frames;

  for (int i = 0; i < (int)map->frames.size(); ++i) {
    Frame* frame = map->frames[map->frames.size() - i - 1].get();

    if (i < num_to_solve)
      frames[frame] = false;
    else if ( i < num_to_present) 
      frames[frame] = true;
    else
      break;
  }

  if (!SetupProblem(range, frames))
    return false;

  problem_->SetParameterBlockConstant(map->cameras[0]->k);
  problem_->SetParameterBlockConstant(map->cameras[1]->k);

  return Run(false);
}


// Solve every frame.
bool Slam::SolveAllFrames(
      LocalMap* map,
      double range,
      bool solve_cameras) {
  std::map<Frame*, bool> frames;
  for (const auto& frame : map->frames) {
    frames[frame.get()] = false;
  }

  if (!SetupProblem(range, frames))
    return false;

  if (!solve_cameras) {
    problem_->SetParameterBlockConstant(map->cameras[0]->k);
    problem_->SetParameterBlockConstant(map->cameras[1]->k);
  } else {
    auto loss = new ceres::CauchyLoss(5);
    for (auto& cam : map->cameras) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<CameraStabilization, 7, 7>(
              new CameraStabilization());
      problem_->AddResidualBlock(cost_function,
                                loss /* squared loss */,
                                cam->k);
    }

#if 0
    problem_->SetParameterBlockConstant(map->frames[0]->rotation().coeffs().data());
    problem_->SetParameterBlockConstant(map->frames[0]->translation().data());
    problem_->SetParameterBlockConstant(map->frames[1]->translation().data());
#endif
  }
  return Run(solve_cameras);
}

bool Slam::Run(bool fine) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.

  ceres::Solver::Options options;

  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  //options.minimizer_progress_to_stdout = true;
  //options.use_inner_iterations = true;
  options.max_num_iterations = 1000;
  options.function_tolerance = 1e-7;

  if (fine) {
    options.max_num_iterations = 1000;
    options.function_tolerance = 1e-9;
  }
  //  if (frames > 15) {
  //  options.use_nonmonotonic_steps = true;
  //  }
  //options.use_block_amd = true;
  //options.num_threads = 3;
  //options.parameter_tolerance = 1e-9;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  if (fine) {
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
  for (auto& frame : map->frames) {
    for (auto& o : frame->observations()) {
      o->error = o->pt;

      ReprojectionError project(o->pt);

      bool result = project(
              frame->rotation().coeffs().data(),
              frame->translation().data(),
              frame->camera()->k,
              o->point->location().data(),
              o->error.data());
      if (!result) {
        printf("Point reporjection failed\n");
        continue;  // Point can't be projected?
      }
      mean = mean + (o->error.norm() - mean) / (count + 1);
      ++count;
    }
  }
  return mean;
}

