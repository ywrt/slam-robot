#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/ordered_groups.h>

#include <vector>
#include <map>

#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace cv;
using namespace Eigen;

// 1. Merge RealPoint and DescribedPoint.
// 2. Maintain a list of observations per point.
// 3. Check visibility before attempting to match?
// 4. Better point tracking: patch matching?
// 5. Downscale to reduce impact of compression.




void Process(char* filename) {
}

struct SnavelyReprojectionError {

  SnavelyReprojectionError(double observed_x, double observed_y)
  : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const scale,
      const T* const intrinsics,
      const T* const camera_rotation,
      const T* const camera_translation,
      const T* const point,
      T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::QuaternionRotatePoint(camera_rotation, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera_translation[0] * point[3];
    p[1] += camera_translation[1] * point[3];
    p[2] += camera_translation[2] * point[3];
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely’s Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
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

// Position and orientation of a frame camera.
struct Frame {
  Frame() :
    rotation_ {1,0,0,0},
    translation_ {0,0,10} {}

    double* rotation() { return rotation_.data(); }
    double* translation() { return translation_.data(); }
    const double* rotation() const { return rotation_.data(); }
    const double* translation() const { return translation_.data(); }

    Vector4d rotation_;
    Vector3d translation_;
    // 0,1,2,3 == Quaternion
    // 4,5,6 == Camera translation
};

// Camera instrinsics.
struct Camera {
  Camera() : data {1.3, -0.02, 0} {}
  double data[3];
  // 0 == focal length.
  // 1, 2 == radial distortion
  double* scale() { return &data[0]; }
  const double* scale() const { return &data[0]; }
  double* instrinsics() { return &data[1]; }
  const double* instrinsics() const { return &data[1]; }
};

struct Observation {
  Vector2d pt;
  int frame_ref;
};

struct Descriptor {
  Descriptor(const uint8_t* v) {
    uint32_t* vv = (uint32_t*) v;
    for (int i = 0; i < 16; ++i) {
      data[i] = vv[i];
    }
  }
  int distance(const uint8_t* v) const {
    uint32_t* vv = (uint32_t*) v;
    int bits = 0;
    for (int i = 0; i < 16; ++i) {
      uint32_t d = data[i] ^ vv[i];
      int count = __builtin_popcount(d);
      bits += count;
    }
    return bits;
  }
  int distance(const Descriptor& desc) const {
    return distance((uint8_t*)desc.data);
  }

  uint32_t data[16];
};

// A patch from a particular frame.
// Used for accurate cross-frame matching.
struct Patch {
  Vector2d pt;
  uint8_t data[64];
  int frame_ref;
};

// A fully tracked point.
// Has the location in world-space, the descriptors, and any
// observations.
struct TrackedPoint {
  TrackedPoint() :
    location_ { 0, 0, 0, 1},
    bad_(false)
    { }

    const double* location() const { return location_.data(); }
    double* location() { return location_.data(); }

    int last_frame() const {
      if (observations_.size() < 1)
        return -1;
      return observations_.back().frame_ref;
    }
    const Vector2d& last_point() const {
      return observations_.back().pt;
    }

    int num_observations() const { return observations_.size(); }

    // Match a descriptor against this point. 'max_distance'
    // is the maximum descriptor distance allowed to succeed.
    // If there's a match, return true and update 'max_distance'
    // to the found distance.
    bool match(const Descriptor& desc, int* max_distance) {
      bool matched = false;
      for (const auto& d : descriptors_) {
        int distance = d.distance(desc);
        if (distance > *max_distance)
          continue;
        *max_distance = distance;
        matched = true;
      }
      return matched;
    }

    Vector4d location_;  // Homogeneous location in world.
    vector<Observation> observations_;
    vector<Descriptor> descriptors_;
    bool bad_;
};

struct LocalMap {
  Camera camera;
  vector<Frame> frames;
  vector<TrackedPoint> points;
  vector<Observation> obs;
};


void Project(
    const Camera& camera,
    const Frame& frame,
    const TrackedPoint& point,
    double* result) {
  SnavelyReprojectionError p(result[0],result[1]);
  p(camera.scale(), camera.instrinsics(),
      frame.rotation(),
      frame.translation(),
      point.location(),
      result);
}


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

void RunSlam(LocalMap* map, int min_frame_to_solve) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  ceres::Solver::Options options;

  const int frame = map->frames.size() - 1;

  set<Frame*> frame_set;
  set<TrackedPoint*> point_set;

  auto loss = new ceres::CauchyLoss(.005);
  //auto loss = new ceres::HuberLoss(0.02);

  for (auto& point : map->points) {
    if (point.observations_.size() < 2)
      continue;
    for (const auto& o : point.observations_) {
      CHECK_GE(o.frame_ref, 0);
      CHECK_LT(o.frame_ref, map->frames.size());

      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 1, 2, 4, 3, 4>(
              new SnavelyReprojectionError(o.pt(0), o.pt(1)));

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

#if 0
  options.use_inner_iterations = true;
  options.inner_iteration_ordering =
      new ceres::ParameterBlockOrdering;
  for (RealPoint& p : map->points) {
    options.inner_iteration_ordering->AddElementToGroup(
        &(p.data[0]), 0);
  }
#endif

#if 1
  ceres::ParameterBlockOrdering* ordering =
      new ceres::ParameterBlockOrdering;

  for (TrackedPoint& p : map->points) {
    ordering->AddElementToGroup(p.location(), 0);
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
  if (frame < 100)
    problem.SetParameterBlockConstant(&(map->camera.data[0]));
  if (frame < 200)
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
  options.max_num_iterations = 5000;
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

void UpdateMap(LocalMap* map,
    int frame,
    const vector<Vector2d>& key_points,
    const Mat& descriptors) {

  CHECK_EQ(key_points.size(), descriptors.rows);
  CHECK_EQ(map->frames.size(), frame);

  if (map->frames.size() > 3) {
    auto s = map->frames.size();
    auto& f1 = map->frames[s - 1];
    auto& f2 = map->frames[s - 2];
    Vector3d motion = f1.translation_ - f2.translation_;
    if (motion.norm() > 1)
      motion /= motion.norm();

    Frame f = f1;
    f.translation_ += motion;
    map->frames.push_back(f);
  } else if (map->frames.size() > 0) {
    map->frames.push_back(map->frames.back());
    map->frames.back().translation()[0] += 0.1;
  } else {
    map->frames.push_back(Frame());
  }

  int hist[512] = {0,};

  // Compute approximate point visibility.
  set<TrackedPoint*> points;
  for (auto& point : map->points) {
    if (point.bad_)
      continue;
    points.insert(&point);  // TODO: actually compute visibility.
  }

  // Match tracked points into the current frame.
  for (int i = 0; i < descriptors.rows;++i) {
    uint8_t* ptr = (uint8_t*) descriptors.ptr(i);
    Descriptor desc(ptr);

    int min_distance = 512;
    TrackedPoint* best_match = NULL;
    for (auto& point : points) {
      if (point->match(ptr, &min_distance))
        best_match = point;
    }
    if (min_distance < 512)
      hist[min_distance/10]++;
    //printf("%3d: dist %d (%d)\n", i, min_distance, best_match);

    if (min_distance < 30) {
      CHECK(best_match != NULL);
      if (best_match->last_frame() == frame) {
        // We've already matched this point in this frame!
        // Mark the point as bad.
        best_match->bad_ = true;
        continue;
      }

      // A useful match
      Observation o;
      o.pt = key_points[i];
      o.frame_ref = frame;
      best_match->observations_.push_back(o);
    } else if (min_distance > 15) {
      // No match and a distinct point.
      // insert this as a new image point.
      map->points.push_back(TrackedPoint());
      TrackedPoint* point = &map->points.back();
      Observation o;
      o.pt = key_points[i];
      o.frame_ref = frame;
      point->observations_.push_back(o);
      point->descriptors_.push_back(desc);
    }
  }

  // Remove tracked points that aren't useful. I.e.
  // tracked points that matched only one frame, and were
  // created more than 3 frames ago.
  // TODO: change map->points to be a vector of points
  // and have this erase bad points.
  for (auto& point : map->points) {
    if (point.num_observations() > 1)
      continue;
    if ((frame - point.last_frame()) < 3)
      continue;
    point.bad_ = true;
  }

  for (size_t i = 0; i < 512; ++i) {
    if (!hist[i])
      continue;
    printf("%3zd: %d\n", i, hist[i]);
  }
}

void NormMap(LocalMap* map) {
  double sum(0);
  double count(0);
  for (auto& f : map->frames) {
    sum += f.translation()[2] * f.translation()[2];
    count++;
  }
  if (sum < 1)
    return;
  double scale = count / sum;
#if 0
  for (auto& p : map->points) {
    p.data[0] *= scale;
    p.data[1] *= scale;
    p.data[2] *= scale;
  }
#endif
  for (auto& f : map->frames) {
    f.translation()[0] *= scale;
    f.translation()[1] *= scale;
    f.translation()[2] *= scale;
  }
}

void DumpMap(LocalMap* map) {
  // SortObs sorter;
  // sort(map->obs.begin(), map->obs.end(), sorter);
  int err_hist[10] = {0,0,0,0,0,0,0,0,0,0};


  for (auto& point : map->points) {
    for (auto& o : point.observations_) {
      Vector2d r(o.pt);

      Project(
          map->camera,
          map->frames[o.frame_ref],
          point,
          r.data());
      double err = r.norm() * 1000;
      if (err < 10) {
        ++err_hist[(int)err];
        continue;
      }
      printf("frame %3d : (matches %d) [%7.3f %7.3f] (%7.2f,%7.2f) -> %.2f\n",
          o.frame_ref,
          point.num_observations(),
          o.pt(0), o.pt(1),
          r[0] * 1000, r[1] * 1000,
          err);
    }
  }
  for (size_t i = 0; i < 10; ++i) {
    printf("err_hist: %2ld : %5d\n", i, err_hist[i]);
  }
  for (auto& f : map->frames) {
    printf("(%f,%f,%f,%f) -> (%f, %f, %f)\n",
        f.rotation()[0], f.rotation()[1], f.rotation()[2], f.rotation()[3],
        f.translation()[0], f.translation()[1], f.translation()[2]);
  }
#if 0
  for (auto& p : map->points) {
    printf("pt %f,%f,%f\n",
        p.data[0], p.data[1], p.data[2]);
  }
#endif

  printf("focal %f r1 %f r2 %f\n",
      map->camera.data[0],
      map->camera.data[1],
      map->camera.data[2]);
}

void DrawCross(cv::Mat* out, const Vector2d& point, int size, Scalar color) {
  Point2f a(
      (point(0) + 1)/2 * out->cols,
      (point(1) + 1)/2 * out->rows);
  line(*out, a - Point2f(size,size), a + Point2f(size,size), color, 1, 8);
  line(*out, a - Point2f(size,-size), a + Point2f(size,-size), color, 1, 8);
}

void DrawLine(cv::Mat* out,
    const Vector2d& from,
    const Vector2d& to,
    Scalar color) {
  Point2f a(
      (from(0) + 1)/2 * out->cols,
      (from(1) + 1)/2 * out->rows);
  Point2f b(
      (to(0) + 1)/2 * out->cols,
      (to(1) + 1)/2 * out->rows);
  line(*out, a, b, color, 1, 8);
}


int main(int argc, char*argv[]) {
  if (argc < 2) {
    fprintf(stderr, "Usage: slam filename\n");
    exit(1);
  }

  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.

  cv::VideoCapture vid(argv[1]);
  cv::Mat img;
  cv::Mat t;
  cv::Mat out;
  cv::Mat grey;

  Ptr<FeatureDetector> detector =
      new GridAdaptedFeatureDetector(
          new PyramidAdaptedFeatureDetector(
              FeatureDetector::create("FAST")),
              150);
  ORB extractor(150, 1.3, 3);

  int frame = -1;

  LocalMap map;

  while (vid.read(img)) {
    frame++;

    cv::transpose(img, t);
    cv::resize(t, out, Size(t.cols / 2, t.rows / 2));
    cv::cvtColor(out, grey, CV_RGB2GRAY);

    std::vector<KeyPoint> keypoints;
    Mat descriptors;

    detector->detect(grey, keypoints);
    extractor.compute(grey, keypoints, descriptors);

    vector<Vector2d> normed_points;
    Vector2d scale;
    scale[0] = out.cols;
    scale[1] = out.rows;
    for (auto& k : keypoints) {
      Vector2d p;
      p[0] = k.pt.x;
      p[1] = k.pt.y;

      p = 1 - 2 * p.array() / scale.array();
      normed_points.push_back(p);
    }

    UpdateMap(&map, frame, normed_points, descriptors);

    for (const auto& point : map.points) {
      if (point.last_frame() != frame)
        continue;
      if (point.num_observations() == 1) {
        DrawCross(&out, point.last_point(), 2, Scalar(0,255,0));
        continue;
      }
      int num = point.observations_.size();
      for (int i = 0; i < (num - 1); ++i) {

        DrawLine(&out, point.observations_[i].pt,
            point.observations_[i+1].pt, Scalar(0,0,0));
      }
      DrawCross(&out, point.last_point(), 5, Scalar(0,0,255));
    }

    cv::imshow( "Display window", out);

    int mod = 3;
    if (frame > 20)
      mod = 6;
    if (frame > 50)
      mod = 15;

    if ((frame%mod) == 0) {
      //  RunSlam(&map, frame - mod);
      //DumpMap(&map);
      // RunSlam(&map, -1);
      //  DumpMap(&map);
      cv::waitKey(0);
    }



  }
  return 0;
}
