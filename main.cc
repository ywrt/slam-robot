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

using namespace std;
using namespace cv;

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
    p[0] += camera_translation[0];
    p[1] += camera_translation[1];
    p[2] += camera_translation[2];
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

struct Frame {
  Frame() :
    rotation_ {1,0,0,0},
    translation_ {0,0,10} {}

    double* rotation() { return &rotation_[0]; }
    double* translation() { return &translation_[0]; }

    double rotation_[4];
    double translation_[3];
    // 0,1,2,3 == Quaternion
    // 4,5,6 == Camera translation
};

struct Camera {
  Camera() : data {.683, -0.02, 0} {}
  double data[3];
  // 0 == focal length.
  // 1, 2 == radial distortion
};

struct RealPoint {
  RealPoint() : data { 0, 0, 0 } {}
  double data[3];
};

struct Observation {
  Point2f pt;
  int point_ref;
  int frame_ref;
  int desc_ref;
  int prev_obs;
};

struct SortObs {
  bool operator()(const Observation& a, const Observation& b) {
    if (a.point_ref != b.point_ref)
      return a.point_ref < b.point_ref;
    if (a.frame_ref != b.frame_ref)
      return a.frame_ref < b.frame_ref;
    return a.pt.x < b.pt.x;
  }
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

  uint32_t data[16];
};

struct DescribedPoint {
  DescribedPoint(const Descriptor& d, int frame, Point2f p) :
    desc(d),
    last_frame(frame),
    last_point(p),
    point_ref(-1),
    matches(0),
    last_obs(-1),
    bad(false) {}

  Descriptor desc;
  int last_frame;
  Point2f last_point;
  int point_ref;
  int matches;
  int last_obs;
  bool bad;
};

struct Map {
  Camera camera;
  vector<Frame> frames;
  vector<RealPoint> points;
  vector<Observation> obs;
  vector<DescribedPoint> descs;
};



void RunSlam(Map* map) {
  // Create residuals for each observation in the bundle adjustment problem. The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;
  ceres::Solver::Options options;

  set<Frame*> frame_set;

  auto loss = new ceres::CauchyLoss(.01);

  for (const auto& o : map->obs) {
    CHECK_GE(o.frame_ref, 0);
    CHECK_GE(o.point_ref, 0);
    CHECK_LT(o.frame_ref, map->frames.size());
    CHECK_LT(o.point_ref, map->points.size());

    Point2f pt = o.pt;
    pt.x = 1. - pt.x / 1024. * 2.;
    pt.y = 1. - pt.y / 768. * 2.;

    // Each Residual block takes a point and a camera as input and outputs a 2
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 1, 2, 4, 3, 3>(
            new SnavelyReprojectionError(pt.x, pt.y));

    problem.AddResidualBlock(cost_function,
        loss /* squared loss */ ,
        &(map->camera.data[0]),
        &(map->camera.data[1]),
        map->frames[o.frame_ref].rotation(),
        map->frames[o.frame_ref].translation(),
        &(map->points[o.point_ref].data[0]));
    Frame* f = &(map->frames[o.frame_ref]);
    frame_set.insert(f);
  }

  if (frame_set.size() < 2)
    return;

  int frames = frame_set.size();

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

  for (RealPoint& p : map->points) {
    ordering->AddElementToGroup(&(p.data[0]), 0);
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

  problem.SetParameterBlockConstant(map->frames[0].translation());
  problem.SetParameterBlockConstant(map->frames[0].rotation());
  //problem.SetParameterBlockConstant(&(map->camera.data[0]));
  if (frames < 12)
    problem.SetParameterBlockConstant(&(map->camera.data[1]));

  if ((frames%12) == 0) {
    for (int i = 1; (i + 2) < (map->frames.size() / 2);  ++i) {
      auto f = &(map->frames[i]);
      if (!frame_set.count(f))
        continue;
      problem.SetParameterBlockConstant(f->translation());
    }
  }

  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
  options.minimizer_progress_to_stdout = true;
  //options.use_inner_iterations = true;
  options.max_num_iterations = 5000;
  options.function_tolerance = 1e-7;
//  if (frames > 15) {
//  options.use_nonmonotonic_steps = true;
//  }
  options.use_block_amd = true;


  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}

void UpdateMap(Map* map,
    int frame,
    const vector<KeyPoint>& key_points,
    const Mat& descriptors) {

  CHECK_EQ(key_points.size(), descriptors.rows);
  CHECK_EQ(map->frames.size(), frame);

  if (map->frames.size() > 3) {
    auto s = map->frames.size();
    auto& f1 = map->frames[s - 1];
    auto& f2 = map->frames[s - 2];
    double motion[3];
    motion[0] = f1.translation()[0] - f2.translation()[0];
    motion[1] = f1.translation()[1] - f2.translation()[1];
    motion[2] = f1.translation()[2] - f2.translation()[2];
    Frame f = f1;
    f.translation()[0] += motion[0];
    f.translation()[1] += motion[1];
    f.translation()[2] += motion[2];
    map->frames.push_back(f);
  } else if (map->frames.size() > 0) {
    map->frames.push_back(map->frames.back());
    map->frames.back().translation()[0] += 0.1;
  } else {
    map->frames.push_back(Frame());
  }


  for (int i = 0; i < descriptors.rows;++i) {
    uint8_t* ptr = (uint8_t*) descriptors.ptr(i);
    int min_distance = 512;
    int best_match = -1;
    for (int di = 0; di < map->descs.size(); ++di) {
      auto& d = map->descs[di];
      if ((frame - d.last_frame) > 15)
        continue;
      int dist = d.desc.distance(ptr);
      if (dist < min_distance) {
        min_distance = dist;
        best_match = di;
      }
    }
    printf("%3d: dist %d (%d)\n", i, min_distance, best_match);

    if (min_distance < 12) {
      DescribedPoint* dp = &(map->descs[best_match]);
      if (dp->last_frame == frame) {
        // A bad point. matches in it's own frame.
        dp->bad = true;
      }
      if (dp->bad)
        continue;
      // A useful match
      // First, ensure the original described point is inserted.

      if (dp->point_ref < 0) {
        dp->point_ref = map->points.size();
        map->points.push_back(RealPoint());

        dp->matches = 1;
        dp->last_obs = map->obs.size();

        Observation o;
        o.pt = dp->last_point;
        o.frame_ref = dp->last_frame;
        o.point_ref = dp->point_ref;
        o.desc_ref = best_match;
        o.prev_obs = -1;
        map->obs.push_back(o);

      }
      Observation o;
      o.pt = key_points[i].pt;
      o.frame_ref = frame;
      o.point_ref = dp->point_ref;
      o.desc_ref = best_match;
      o.prev_obs = dp->last_obs;

      dp->last_frame = frame;
      dp->last_point = o.pt;
      dp->last_obs = map->obs.size();
      ++dp->matches;

      map->obs.push_back(o);
    } else {
      // No match and a distinct point.
      // insert this as a new image point.
      map->descs.push_back(
          DescribedPoint(
              Descriptor(ptr),
              frame,
              key_points[i].pt));
    }
  }
}

void NormMap(Map* map) {
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


void DumpMap(Map* map) {
  // SortObs sorter;
  // sort(map->obs.begin(), map->obs.end(), sorter);
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


int main(int argc, char*argv[]) {
  if (argc < 2) {
    fprintf(stderr, "Usage: slam filename\n");
    exit(1);
  }

  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.

  cv::VideoCapture vid(argv[1]);
  cv::Mat img;
  cv::Mat out;
  cv::Mat grey;

  Ptr<FeatureDetector> detector(new ORB());
  Ptr<DescriptorExtractor> extractor(DescriptorExtractor::create("FREAK"));

  int frame = 0;

  Map map;

  while (vid.read(img)) {
    cv::transpose(img, out);
    cv::cvtColor(out, grey, CV_RGB2GRAY);

    std::vector<KeyPoint> keypoints;
    detector->detect(grey, keypoints);

    Mat descriptors;
    extractor->compute(grey, keypoints, descriptors);

    UpdateMap(&map, frame, keypoints, descriptors);

    for (auto& dp : map.descs) {
      if (dp.point_ref >= 0)
        continue;
      if ((frame - dp.last_frame) > 5)
        continue;
      const Point2f&a = dp.last_point;
      auto c = Scalar(0,255,0);
      line(out, a - Point2f(2,2), a + Point2f(2,2), c, 1, 8);
      line(out, a - Point2f(2,-2), a + Point2f(2,-2), c, 1, 8);
    }


    for (const Observation& o : map.obs) {
      if (o.frame_ref != frame)
        continue;

      auto c = Scalar(0,0,255);
      const Point2f& a = o.pt;
      line(out, a - Point2f(5,5), a + Point2f(5,5), c, 2, 8);
      line(out, a - Point2f(5,-5), a + Point2f(5,-5), c, 2, 8);

      for (auto x = &o; x->prev_obs >= 0 ; x = &(map.obs[x->prev_obs])) {
        auto px = &(map.obs[x->prev_obs]);
        const Point2f& a = x->pt;
        const Point2f& b = px->pt;
        line(out, a, b, Scalar(0,0,0), 1, 8);
      }

    }

    DumpMap(&map);

    if (frame < 20) {
      // map.camera = Camera();
    }
    if ((frame%3) == 0) {
      //NormMap(&map);
      RunSlam(&map);
      DumpMap(&map);

    }

    cv::imshow( "Display window", out);
    cv::waitKey(0);
    frame++;
  }
  return 0;
}
