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

double ShowMatch(const Mat& a_img, const Mat& b_img, const Point2f& a, const Point2f& b, const Size& window) {
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
  int lost(0), fail(0), good(0);
  for (unsigned int i = 0; i < status1.size(); ++i) {
    if (!status1[i] || !status2[i]) {
      ++lost;
      continue;
    }
    if (norm(in[i] - in1[i]) > 2) {
      ++fail;
      continue;
    }

    double score = ScoreMatch(prev[0], img[0], in[i], out[i], Size(21, 21));
    printf("%3d : Score %f\n", list[i].id, score);
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

  cout << "Lost " << lost << ", Fail " << fail << ", Good " << good << "\n";
  return result;
}

template<typename FUNC>
void AddNewFeatures(const Mat& img, FeatureList* list, FUNC get_next_id) {
  vector<Point2f> corners;
  goodFeaturesToTrack(img,
      corners,
      100,  // Max corners.
      0.05,  // Max quality ratio.
      10  // Minimum distance between features.
      );

  const int size = 20;
  int grid[size + 2][size + 2];
  for (const auto& f : *list) {
    int gx = (f.pt.x / img.size().width) * size + 1;
    int gy = (f.pt.y / img.size().height) * size + 1;
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
    if (f.point && f.point->bad_ > 2)
      continue;
    result.push_back(f);
  }
  return result;
}


}  // namespace

bool Matcher::Track(const Mat& img, int camera, LocalMap* map) {
  auto&d = *data_;
  LOG(ERROR) << "Track";


  CHECK_NE(img.size().width, 0);
  CHECK_NE(img.size().height, 0);
  CHECK_NOTNULL(map);

  vector<Mat> pyr;
  buildOpticalFlowPyramid(img, pyr, Size(kWindowSize, kWindowSize), 5, true); 

  if (!d.img2_.size()) {
    d.img2_ = move(d.img1_);
    d.f2_ = move(d.f1_);

    d.img1_ = move(pyr);
    d.f1_.clear();

    return true;
  }

  Frame* frame = map->AddFrame(map->cameras[camera].get());
  CHECK_NOTNULL(frame);

  auto list1 = RunTrack(d.img1_, pyr, FilterBad(d.f1_), 400);
  auto list2 = RunTrack(d.img2_, pyr, FilterBad(d.f2_), 300);

  auto list = MergeLists(list2, list1, 5);

  AddNewFeatures(img, &list, [&d]() -> int { return d.next_fid++; });

  for (auto&f : list) {
    Vector2d frame_point;
    frame_point << f.pt.x / img.size().width * 2 - 1, f.pt.y / img.size().height * 2 - 1;
    if (!f.point) {
      Vector4d location = map->frame(frame->frame_num)->Unproject(frame_point, 500);
      f.point = map->AddPoint(f.id, location);
    }

    f.point->AddObservation({frame_point, frame->frame_num});
  }

  d.img2_ = move(d.img1_);
  d.f2_ = move(d.f1_);

  d.img1_ = move(pyr);
  d.f1_ = move(list);


#if 0
  double ransacReprojThreshold = 10;


  // LK tracker previous features.

  // Detect corners.
  CHECK_EQ(CV_8U, img.type());
  vector<KeyPoint> keypoints1;
  detector_->detect(img, keypoints1, Mat());

  // Compute descriptors for each corner.
  Mat descriptors1;
  extractor_->compute(img, keypoints1, descriptors1);

  // Search for matching descriptors.
  vector<bool> matched;  // was the corner successfully matched.
  matched.resize(keypoints1.size());
  vector<DMatch> matches;
  if (descriptors_->size().height > 0 && descriptors1.size().height > 0) {
    LOG(ERROR) << "query " << descriptors1.size().height << " against "
               << descriptors_->size().height;
    CHECK_EQ(descriptors1.size().width, descriptors_->size().width);
    matcher_->match(descriptors1, *(descriptors_.get()), matches, Mat());
  }

  // Find the homography
  vector<Point2d> p1;
  vector<Point2d> p2;
  for (const auto& m : matches) {
    const Data& d = data_[m.trainIdx];

    matched[m.queryIdx] = true;

    Vector2d projected;
    if (!frame->Project(d.point->location(), &projected))
      continue;

    p1.push_back({(projected(0) + 1) / 2 * img.size().width, (projected(1) + 1) /2 * img.size().height });
    p2.push_back(keypoints1[m.queryIdx].pt);
  }
  
  if (p1.size() > 6) {
    Mat homog = findHomography(Mat(p1), Mat(p2), RANSAC, ransacReprojThreshold);

    // Accept points that match the homography.
    Mat p1t;
    perspectiveTransform(Mat(p1), p1t, homog);
    for (unsigned int i = 0; i < matches.size(); ++i) {
      if (0 && norm(p2[i] - p1t.at<Point2d>(i, 0)) > ransacReprojThreshold) // outlier
        continue;

      Vector2d frame_point;
      frame_point << p2[i].x / img.size().width * 2 - 1, p2[i].y / img.size().height * 2 - 1;

      Observation o;
      o.frame_idx = frame->frame_num;
      o.pt = frame_point;

      data_[matches[i].trainIdx].point->AddObservation(o);
    }
  }


  const int size = 30;
  int grid[size][size] = { };

  

  // Add unmatched points as new descriptors.
  for (unsigned int i = 0; i < keypoints1.size(); ++i) {
    if (matched[i])
      continue;

    Vector2d frame_point;
    frame_point << keypoints1[i].pt.x / img.size().width * 2 - 1, keypoints1[i].pt.y / img.size().height * 2 - 1;
    Vector4d location = map->frame(frame->frame_num)->Unproject(frame_point, 500);
    TrackedPoint* point = map->AddPoint(location);

    point->AddObservation({frame_point, frame->frame_num});

    data_.push_back({point, frame->frame_num});

    descriptors_->push_back(descriptors1.row(i));
  }
#endif
  return true;
}
