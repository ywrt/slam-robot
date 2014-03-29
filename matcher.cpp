//
// Switch to using the LK tracker.
// Initialize with 'GoodFeaturesToTrack'.
// Supress points that are already being tracked.
// 
#include <vector>
#include <set>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "localmap.h"

#include "matcher.h"

using namespace cv;
using namespace std;

static const int kWindowSize = 11;

struct Matcher::Data {
  Data() : next_fid(0) {}

  struct Feature {
    Point2f pt;
    float response;
    int id;
    TrackedPoint* point;
  };

  vector<Feature> f1_;
  vector<Feature> f2_;

  vector<Mat> img1_;
  vector<Mat> img2_;

  int next_fid;
};

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
  patch_a = patch_a + (cv::Scalar::all(128) - cv::mean(patch_a));
//  normalize(patch_a, patch_a, 128 * window.width * window.height, NORM_L1);

  cv::Mat patch_b;
  cv::getRectSubPix(b_img, window, b, patch_b);
  patch_b = patch_b + (cv::Scalar::all(128) - cv::mean(patch_b));
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

FeatureList RunTrack(const ImageStack& prev, const ImageStack& img, const FeatureList& list, int max_error) {
  if (!list.size())
    return FeatureList();

  vector<Point2f> in, in1, out;
  for (const auto& f : list) {
    in.push_back(f.pt);
    out.push_back(f.pt);
  }

  // Forward match.
  vector<unsigned char> status1(in.size());
  vector<float> err1(in.size());
  calcOpticalFlowPyrLK(prev, img, in, out, status1, err1, Size(kWindowSize, kWindowSize), 5,
      TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
      OPTFLOW_USE_INITIAL_FLOW | OPTFLOW_LK_GET_MIN_EIGENVALS,
      1e-3);

  // Reverse match.
  vector<unsigned char> status2(in.size());
  vector<float> err2(in.size());
  calcOpticalFlowPyrLK(img, prev, out, in1, status2, err2, Size(kWindowSize, kWindowSize), 5,
      TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
      0,
      1e-3);

  FeatureList result;
  int lost(0), fail(0), good(0), oob(0);
  Size img_size = img[0].size();
  Rect bounds(Point(1, 1), Size(img_size.width - 2, img_size.height - 2));
  for (unsigned int i = 0; i < status1.size(); ++i) {
    if (!status1[i] || !status2[i]) {
      ++lost;
      continue;
    }
    if (norm(in[i] - in1[i]) > 2) {
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
      fail++;
      continue;
    }

    result.push_back(list[i]);
    result.back().pt = out[i];
    result.back().response = err1[i];
    ++good;
  }

  cout << "Lost " << lost << ", Fail " << fail
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
      0.02,  // Max quality ratio.
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
    f.response = 0;
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
bool Matcher::Track(const Mat& img, Frame* frame, LocalMap* map) {
  auto&d = *data_;

  CHECK_NE(img.size().width, 0);
  CHECK_NE(img.size().height, 0);
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(frame);

  // Build an image pyamid for the new image.
  vector<Mat> pyr;
  buildOpticalFlowPyramid(img, pyr, Size(kWindowSize, kWindowSize), 5, true); 

  FeatureList list;
  // Track against the previous image
  // TODO: Lift out constants.
  if (d.img1_.size()) {
    list = RunTrack(d.img1_, pyr, FilterBad(d.f1_), 400);
  }

  // Track against the previous previous image
  if (d.img2_.size()) {
    auto list2 = RunTrack(d.img2_, pyr, FilterBad(d.f2_), 300);
    list = MergeLists(list2, list, 5);
  }

  // If there are insufficient trackable features, add new features from the
  // current image that aren't too close to an existing feature. Pass in a
  // lambda to generate feature IDs.
  // TODO: Lift out constant.
  if (list.size() < 40) {
    AddNewFeatures(img, &list, [&d]() -> int { return d.next_fid++; });
  }

  // Add the new observations to the LocalMap. If this feature doesn't
  // have an associated TrackedPoint then add one. Use Frame::Unproject
  // to initialize the world space location of the tracked point.
  for (auto&f : list) {
    Vector2d fpt;
    fpt << f.pt.x, f.pt.y;
    Vector2d frame_point = frame->camera()->Undistort(fpt);

    if (!f.point) {
      // TODO: Lift constant.
      auto location = frame->Unproject(frame_point, 1500);
      f.point = map->AddPoint(f.id, location);
    }

    f.point->AddObservation({frame_point, frame});
  }

  // Move the slide window forward.
  d.img2_ = move(d.img1_);
  d.f2_ = move(d.f1_);

  d.img1_ = move(pyr);
  d.f1_ = move(list);
  return true;
}
