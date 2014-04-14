//
// Switch to using the LK tracker.
// Initialize with 'GoodFeaturesToTrack'.
// Supress points that are already being tracked.
// 
#include <vector>
#include <set>
#include <map>
#include <deque>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "localmap.h"
#include "klt.h"

#include "matcher.h"


using namespace cv;
using namespace std;

static const int kWindowSize = 13;

extern int debug;

struct Matcher::Data {
  Data() : next_fid(0) {}

  struct Feature {
    Point2f pt;
    Point2f base_pt;
    TrackedPoint* point;

    int id;
  };

  // vector of feature lists, indexed by camera id.
  map<int, vector<Feature>> features;
  map<int, vector<Mat>> images;
  map<int, Mat> originals;

  int next_fid;
};

namespace {

map<int, deque<Mat>> patches;

} // namespace

const map<int, deque<Mat>>& GetPatches() {
  return patches;
}

typedef Matcher::Data::Feature Feature;
typedef vector<Matcher::Data::Feature> FeatureList;
typedef vector<Mat> ImageStack;

Matcher::~Matcher() { }

Matcher::Matcher() : data_(new Data) { }

namespace {

template<typename T>
void VisitPairs(
    const FeatureList& a,
    const FeatureList& b,
    const T& visitor) {

  int ai = 0, bi = 0;
  while (ai < a.size() && bi < b.size()) {
    if (a[ai].id < b[bi].id) {
      ++ai;
      continue;
    }
    if (b[bi].id < a[ai].id) {
      ++bi;
      continue;
    }

    visitor(a[ai], b[bi]);

    ++ai;
    ++bi;
  }
}

void ShowMatch(const Mat& a_img, const Mat& b_img, const Point2f& a, const Point2f& b, const Size& window) {
  cv::Mat patch_a;
  cv::getRectSubPix(a_img, window, a, patch_a);
//  patch_a = patch_a + (cv::Scalar::all(128) - cv::mean(patch_a));
//  normalize(patch_a, patch_a, 128 * window.width * window.height, NORM_L1);

  cv::Mat patch_b;
  cv::getRectSubPix(b_img, window, b, patch_b);
//  patch_b = patch_b + (cv::Scalar::all(128) - cv::mean(patch_b));
//  normalize(patch_b, patch_b, 128 * window.width * window.height, NORM_L1);

  cv::Mat diff;

  cv::absdiff(patch_a, patch_b, diff);

  int scale = 15;
  resize(patch_a, patch_a, patch_a.size() * scale, 0, 0, INTER_NEAREST);
  resize(patch_b, patch_b, patch_b.size() * scale, 0, 0, INTER_NEAREST);
  resize(diff, diff, diff.size() * scale, 0, 0, INTER_NEAREST);
  Size s = patch_a.size();

  Mat out(Size(s.width * 3, s.height), a_img.type());

  patch_a.copyTo(out(Rect(0 * s.width, 0, s.width, s.height)));
  patch_b.copyTo(out(Rect(1 * s.width, 0, s.width, s.height)));
  diff.copyTo(out(Rect(2 * s.width, 0, s.width, s.height)));

  cv::namedWindow("Diff", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::moveWindow("Diff", 1440, 1500);
  cv::imshow("Diff", out);
  cv::waitKey(0);
}


void ShowPatches(
    const vector<KLTTracker::Patch>& p1,
    const vector<KLTTracker::Patch>& p2,
    const vector<KLTTracker::Patch>& p3) {
  const int scale = 15;

  cv::Size s = p1[0].size;

  Mat out(Size((s.width + 2) * 3, (s.height + 2) * p1.size()), CV_32F);
  vector<const vector<KLTTracker::Patch>*> pv;
  pv.push_back(&p1);
  pv.push_back(&p2);
  pv.push_back(&p3);

  out = Scalar(0,0,0);

  for (int x = 0; x < 3; ++x) {
    const auto& p = *pv[x];
    for (unsigned int y = 0; y < p.size(); ++y) {
      const auto& patch = p[y];
      cv::Mat(patch.size.width, patch.size.height, CV_32F, (float*) &(patch.data[0]), patch.size.width * sizeof(float)).
          copyTo(out(Rect(x * (s.width + 1), y * (s.height + 1), s.width, s.height)));
    }
  }

  for (int i = 0; i < p1[0].len; ++i) {
    printf("%8.3f ", p1[0].data[i]);
  }
  printf("\n");

  for (int i = 0; i < p2[0].len; ++i) {
    printf("%8.3f ", p1[0].gradx[i]);
  }
  printf("\n");

  Mat out1;
  resize(out, out1, out.size() * scale, 0, 0, INTER_NEAREST);

  cv::namedWindow("Diff", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::moveWindow("Diff", 1440, 1500);
  cv::imshow("Diff", out1);
  cv::waitKey(0);
}


// Compute score for a patch.
double ScoreMatch(const Mat& a_img, const Mat& b_img, const Point2f& a, const Point2f& b, const Size& window) {
  cv::Mat patch_a;
  cv::getRectSubPix(a_img, window, a, patch_a);
  patch_a = patch_a + (cv::Scalar::all(128) - cv::mean(patch_a));
//  normalize(patch_a, patch_a, 128 * window.width * window.height, NORM_L1);

  cv::Mat patch_b;
  cv::getRectSubPix(b_img, window, b, patch_b);
  patch_b = patch_b + (cv::Scalar::all(128) - cv::mean(patch_b));
//  normalize(patch_b, patch_b, 128 * window.width * window.height, NORM_L1);

  cv::Mat diff;

  cv::absdiff(patch_a, patch_b, diff);

  double m = cv::norm(patch_a, patch_b);

  return m;
}

FeatureList RunTrack1(const Mat& prev, const Mat& img, const FeatureList& list, int max_error) {
  KLTTracker tracker(Size(kWindowSize, kWindowSize));

  auto stack1 = tracker.MakePyramid(prev, 5);
  auto stack2 = tracker.MakePyramid(img, 5);

  const int margin = kWindowSize / 2 + 1;
  Size img_size = img.size();
  Rect bounds(Point(margin, margin), Size(img_size.width - margin, img_size.height - margin));
  FeatureList result;
  int lost(0), fail(0), fail_score(0), good(0), oob(0);
  int iters = 80;
  //if (debug) iters = 1;
  for (auto& f : list) {
    Point2f np = f.pt;
    auto patches1 = tracker.GetPatches(stack1, f.base_pt);
    auto status1 = tracker.TrackFeature(stack2, patches1, 0.00001, iters, &np);
    if (status1) {
      ++lost;
      continue;
    }

    Point2f op = np;
    auto patches2 = tracker.GetPatches(stack2, np);
    auto status2 = tracker.TrackFeature(stack1, patches2, 0.00001, iters, &op);
    if (status2) {
      ++lost;
      continue;
    }

    auto patches3 = tracker.GetPatches(stack1, op);

    printf(" = [%7.2f, %7.2f] => [%7.2f, %7.2f] => [%7.2f, %7.2f]\n", f.base_pt.x, f.base_pt.y, np.x, np.y, op.x, op.y);
    if (debug)
      ShowPatches(patches1, patches3, patches2);

    if (norm(op - f.base_pt) > 0.5) {
      ++fail;
      continue;
    }

    if (!bounds.contains(np)) {
      ++oob;
      continue;
    }

    result.push_back(f);
    result.back().pt = np;
    ++good;
  }

  cout << "Track1: Lost " << lost << ", Fail " << fail
    << ", Bad score " << fail_score
    << ", OOB " << oob
    << ", Good " << good << "\n";
  return result;
}

FeatureList RunTrack(const ImageStack& prev, const ImageStack& img, const FeatureList& list, int max_error) {
  if (!list.size())
    return FeatureList();

  vector<Point2f> in, in1, out;
  for (const auto& f : list) {
    in.push_back(f.base_pt);
    out.push_back(f.pt);
  }

  // Forward match.
  vector<unsigned char> status1(in.size());
  vector<float> err1(in.size());
  calcOpticalFlowPyrLK(prev, img, in, out, status1, err1, Size(kWindowSize, kWindowSize), 5,
      TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 40, 0.001),
      OPTFLOW_USE_INITIAL_FLOW | OPTFLOW_LK_GET_MIN_EIGENVALS,
      1e-3);

  // Reverse match.
  vector<unsigned char> status2(in.size());
  vector<float> err2(in.size());
  calcOpticalFlowPyrLK(img, prev, out, in1, status2, err2, Size(kWindowSize, kWindowSize), 5,
      TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 40, 0.001),
      0,
      1e-3);

  FeatureList result;
  int lost(0), fail(0), fail_score(0), good(0), oob(0);
  Size img_size = img[0].size();
  const int margin = kWindowSize / 2 + 1;
  Rect bounds(Point(margin, margin), Size(img_size.width - margin, img_size.height - margin));
  for (unsigned int i = 0; i < status1.size(); ++i) {
    if (!status1[i] || !status2[i]) {
      ++lost;
      continue;
    }
    // TODO: lift constant.
    if (norm(in[i] - in1[i]) > 0.5) {
      ++fail;
      continue;
    }

    if (!bounds.contains(out[i])) {
      ++oob;
      continue;
    }

    double score = ScoreMatch(prev[0], img[0], in[i], out[i], Size(21, 21));
    //printf("%3d : Score %f\n", list[i].id, score);
    if (score > max_error || list[i].id == 18) {
      //ShowMatch(prev[0], img[0], in[i], out[i], Size(21, 21));
    }

    if (score > max_error) {
      fail_score++;
      continue;
    }

    result.push_back(list[i]);
    result.back().pt = out[i];
    ++good;
  }

  cout << "Lost " << lost << ", Fail " << fail
    << ", Bad score " << fail_score
    << ", OOB " << oob
    << ", Good " << good << "\n";
  return result;
}

template<typename FUNC>
void AddNewFeatures(const Mat& img, FeatureList* list, FUNC get_next_id) {
  vector<Point2f> corners;
  goodFeaturesToTrack(img,
      corners,
      100,  // Max corners.
      0.05,  // Max quality ratio.
      20  // Minimum distance between features.
      );

  const int size = 30;
  int grid[size + 2][size + 2] = {};
  for (const auto& f : *list) {
    int gx = (f.pt.x / img.size().width) * size + 1;
    int gy = (f.pt.y / img.size().height) * size + 1;
    CHECK_LT(0, gx) << f.pt.x << ", " << f.pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
    CHECK_LT(0, gy) << f.pt.x << ", " << f.pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
    CHECK_GT(size + 2, gx) << f.pt.x << ", " << f.pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
    CHECK_GT(size + 2, gy) << f.pt.x << ", " << f.pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
    grid[gx-1][gy-1] = 1;
    grid[gx-0][gy-1] = 1;
    grid[gx+1][gy-1] = 1;
    grid[gx-1][gy-0] = 1;
    grid[gx-0][gy-0] = 1;
    grid[gx+1][gy-0] = 1;
    grid[gx-1][gy+1] = 1;
    grid[gx-0][gy+1] = 1;
    grid[gx+1][gy+1] = 1;
  }

  int added = 0;
  for (const auto& c : corners) {
    int gx = (c.x / img.size().width) * size + 1;
    int gy = (c.y / img.size().height) * size + 1;
    CHECK_LT(0, gx);
    CHECK_LT(0, gy);
    CHECK_GT(size + 2, gx);
    CHECK_GT(size + 2, gy);
    if (grid[gx][gy])
      continue;

    Feature f;
    f.pt = c;
    f.id = get_next_id();
    f.point = nullptr;
    list->push_back(f);
    ++added;
  }

  cout << "Added " << added << " new features\n";
}
// Return the merged list. Merge is done by feature ID (stripping
// features with differing locations), and by location.
FeatureList MergeLists(
    const FeatureList& a,
    const FeatureList& b,
    int max_l1dist) {

  std::set<pair<int, int> > loc_map;
  FeatureList result;

  unsigned int ai = 0, bi = 0;
  while (ai < a.size() && bi < b.size()) {
    auto& aa = a[ai];
    auto& bb = b[bi];

    Feature const * f = nullptr;
    if (aa.id < bb.id) {
      f = &aa;
      ai++;
    } else if (bb.id < aa.id) {
      f = &bb;
      bi++;
    } else {
      f = &aa;
      int l1_dist = abs(aa.pt.x - bb.pt.x) + abs(aa.pt.y - bb.pt.y);
      ++ai;
      ++bi;
      if (l1_dist > max_l1dist) {
        cout << "Removing " << aa.id << " due to error " << l1_dist << " in merge: ("
          << aa.pt.x << ", " << aa.pt.y << ") v ("
          << bb.pt.x << ", " << bb.pt.y << ")"
          << endl;
        continue;
      }
    }

    if (loc_map.count({f->pt.x, f->pt.y})) {
      cout << "Removing " << f->id << " due to duplicate location " << endl;
      continue;
    }

    loc_map.insert({f->pt.x, f->pt.y});
    result.push_back(*f);
  }

  for (; ai < a.size(); ++ai) {
    if (loc_map.count({a[ai].pt.x, a[ai].pt.y})) {
      cout << "Removing a " << a[ai].id << " due to duplicate location " << endl;
      continue;
    }

    loc_map.insert({a[ai].pt.x, a[ai].pt.y});
    result.push_back(a[ai]);
  }

  for (; bi < b.size(); ++bi) {
    if (loc_map.count({b[bi].pt.x, b[bi].pt.y})) {
      cout << "Removing b " << b[bi].id << " due to duplicate location " << endl;
      continue;
    }

    loc_map.insert({b[bi].pt.x, b[bi].pt.y});
    result.push_back(b[bi]);
  }

  return result;
}

// Remove features that reference a bad point.
FeatureList FilterBad(const FeatureList& list) {
  FeatureList result;
  for (auto& f : list) {
    if (f.point && !f.point->feature_usable()) {
      printf("p %3d: not feature usable\n", f.point->id());
      continue;
    }
    result.push_back(f);
  }
  return result;
}


}  // namespace


// Track features from the previous two frames.
// (There may be an assumption that the previous frames are from alternating
// cameras).
//
// Adds matched features to the localmap.
bool Matcher::Track(const Mat& img, Frame* frame, int camera, LocalMap* map) {
  auto&d = *data_;

  CHECK_NE(img.size().width, 0);
  CHECK_NE(img.size().height, 0);
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(frame);

  bool new_keyframe = false;

  // Build an image pyamid for the new image.
  vector<Mat> pyr;
  Mat grey;
  cvtColor(img, grey, CV_RGB2GRAY);
  buildOpticalFlowPyramid(grey, pyr, Size(kWindowSize, kWindowSize), 5, true); 

  FeatureList list;
  // Track against the previous cross-camera image
  // TODO: Lift out constants.
  if (d.images.count(camera ^ 1)) {
    //list = RunTrack(d.images[camera ^ 1], pyr, FilterBad(d.features[camera ^ 1]), 1000);

    list = RunTrack1(d.originals[camera ^ 1], img, FilterBad(d.features[camera ^ 1]), 1000);
  }

  // Track against the previous same-camera image
  if (d.images.count(camera)) {
    //auto list2 = RunTrack(d.images[camera], pyr, FilterBad(d.features[camera]), 1000);
    auto list2 = RunTrack1(d.originals[camera], img, FilterBad(d.features[camera]), 1000);
    list = MergeLists(list2, list, 5);
  }

  // If there are insufficient trackable features, add new features from the
  // current image that aren't too close to an existing feature. Pass in a
  // lambda to generate feature IDs.
  // TODO: Lift out constant.
  if (list.size() < 40) {
    AddNewFeatures(grey, &list, [&d]() -> int { return d.next_fid++; });
    new_keyframe = true;
    printf("Adding new keyframe for camera %d on frame %d\n", camera, frame->id());
  }

  // Add the new observations to the LocalMap. If this feature doesn't
  // have an associated TrackedPoint then add one. Use Frame::Unproject
  // to initialize the world space location of the tracked point.
  for (auto&f : list) {
    Vector2d fpt(f.pt.x, f.pt.y);
    Vector2d frame_point = frame->camera()->Undistort(fpt);
    Vector2d test_point = frame->camera()->Distort(frame_point);
    double test_dist = (fpt - test_point).norm();
    CHECK_NEAR(test_dist, 0, 1e-5);

    if (!f.point) {
      // TODO: Lift constant.
      auto location = frame->Unproject(frame_point / 530., 1500);
      f.point = map->AddPoint(f.id, location);
    }

    f.point->AddObservation({frame_point, frame});

    cv::Mat patch;
    cv::getRectSubPix(img, Size(kWindowSize, kWindowSize), f.pt, patch);
    patches[f.id].push_front(patch.clone());
    if (patches[f.id].size() > 30)
      patches[f.id].pop_back();
  }

  if (new_keyframe) {
    // We changing the reference image, so update the reference point.
    for (auto& f : list) {
      f.base_pt = f.pt;
    }
    d.images[camera] = move(pyr);
    d.features[camera] = move(list);
    d.originals[camera] = img.clone();
  }

  return true;
}
