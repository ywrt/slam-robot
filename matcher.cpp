//
// 1. Store patches rather than reference images.
// 2. Only use projected point when it's reliable?
// 
#include <vector>
#include <set>
#include <map>
#include <deque>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <memory>

#include "localmap.h"
#include "hessian.h"

#include "matcher.h"

typedef HessianTracker FeatureTracker;
typedef FeatureTracker::Pyramid Pyramid;

using namespace cv;
using namespace std;

static const int kWindowSize = 13;

extern int debug;

struct Matcher::Data {
  Data() : next_fid(0) {}

  struct View {
    Frame* frame;
    Pyramid pyramid;
    Mat original;
  };

  struct Feature {
    TrackedPoint* point;
    map<View*, Point2f> matches;
  };

  struct FeatureCmp {
    bool operator()(const unique_ptr<Feature>& a, const unique_ptr<Feature>& b) const {
      return a->point->id() < b->point->id();
    }
  };

  typedef set<unique_ptr<Feature>, FeatureCmp> FeatureSet;
  // Set of live tracked features.
  FeatureSet features;

  // views, indexed by camera id.
  deque<unique_ptr<View>> views;

  int next_fid;
};

typedef Matcher::Data::Feature Feature;
typedef Matcher::Data::View View;
typedef Matcher::Data::Feature Feature;
typedef Matcher::Data::FeatureSet FeatureSet;

namespace {

map<int, deque<Mat>> patches;

} // namespace

const map<int, deque<Mat>>& GetPatches() {
  return patches;
}


Matcher::~Matcher() { }

Matcher::Matcher() : data_(new Data) { }

namespace {

template<typename T>
void ShowPatches(
    const vector<T>& p1,
    const vector<T>& p2,
    const vector<T>& p3) {
  const int scale = 15;

  cv::Size s = p1[0].size;

  Mat out(Size((s.width + 2) * 3, (s.height + 2) * p1.size()), CV_32F);
  vector<const vector<T>*> pv;
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

  Mat out1;
  resize(out, out1, out.size() * scale, 0, 0, INTER_NEAREST);

  cv::namedWindow("Diff", CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::moveWindow("Diff", 1440, 1500);
  cv::imshow("Diff", out1);
  cv::waitKey(0);
}


void AddNewFeatures(const Mat& img, const map<Feature*, Point2f>& matches, vector<Point2f>* result) {
  vector<Point2f> corners;
  goodFeaturesToTrack(img,
      corners,
      100,  // Max corners.
      0.05,  // Max quality ratio.
      20  // Minimum distance between features.
      );

  const int size = 30;
  int grid[size + 2][size + 2] = {};
  for (const auto& m : matches) {
    const Point2f& pt = m.second;
    int gx = (pt.x / img.size().width) * size + 1;
    int gy = (pt.y / img.size().height) * size + 1;
    CHECK_LT(0, gx) << pt.x << ", " << pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
    CHECK_LT(0, gy) << pt.x << ", " << pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
    CHECK_GT(size + 2, gx) << pt.x << ", " << pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
    CHECK_GT(size + 2, gy) << pt.x << ", " << pt.y << " : [" << img.size().width << ", " << img.size().height << "\n";
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

    result->push_back(c);
    ++added;
  }

  cout << "Added " << added << " new features\n";
}

}  // namespace

bool TrackFeature(int id, FeatureTracker* tracker, const View& from, const Point2f& from_pt, const View& to, int lvls, Point2f* to_pt) {
  // Forward match.
  auto p1 = tracker->GetPatches(from.pyramid, from_pt, lvls);
  auto s1 = tracker->TrackFeature(to.pyramid, p1, 0.001, 10, to_pt);

  auto tpt = *to_pt;
  // Then reverse match to check.
  auto p2 = tracker->GetPatches(to.pyramid, *to_pt, lvls);
  Point2f back_pt = from_pt;
  auto s2 = tracker->TrackFeature(from.pyramid, p2, 0.001, 10, &back_pt);

  bool good = (norm(from_pt - back_pt) < 0.3);

  // Debugging
  //printf("%3d: [%7.2f, %7.2f] => [%7.2f, %7.2f] => [%7.2f, %7.2f] => [%7.2f, %7.2f] (%d) %s %s\n",
  //    id, from_pt.x, from_pt.y, tpt.x, tpt.y, to_pt->x, to_pt->y, back_pt.x, back_pt.y, lvls,
  //    good ? "Good" : "Bad", (s1 || s2) ? "Lost" : "");

  // TODO: Move this to be early exit after debugging finishes.
  if (s1 || s2)
    return false;

  if (debug) {
    auto p3 = tracker->GetPatches(from.pyramid, back_pt, lvls);
    ShowPatches(p1, p3, p2);
  }

  // Reject if the reverse differs significantly from the original.
  if (norm(from_pt - back_pt) > 0.3) {
    return false;
  }

  return true;
}

void FindMatches(
    const FeatureSet& features,
    const View& view,
    bool second_pass,
    FeatureTracker* tracker,
    std::map<Feature*, Point2f>* matches) {
  // For each feature, try and propogate forward into this view (if not
  // already done so.

  // If we're on the second pass, we only looking at projectable points,
  // and we're looking doing fine matching.
  int levels = second_pass ? 3 : 6;
  for (auto& f : features) {
    if (matches->count(f.get()))
      continue;
    for (auto& m : f->matches) {
      const auto& from_view = *m.first;
      const auto& from_pt = m.second;
      const auto& to_view = view;
      auto to_pt = from_pt;

      // Use the projected point location as a starting point in
      // searching for a match.
      if (f->point->slam_usable()) {
        Eigen::Vector2d p;
        if (to_view.frame->Project(f->point->location(), &p)) {
          to_pt.x = p(0);
          to_pt.y = p(1);
        }
      } else if (second_pass) {
        continue;  // Only tried and failed on the first pass.
      }

      //debug = (f->point->id() == 40);

      if (to_pt.x < 0 || to_pt.y < 0 || to_pt.x >= view.original.cols || to_pt.y > view.original.rows)
        continue;  // OOBs

      if (!TrackFeature(f->point->id(), tracker, from_view, from_pt, to_view, levels, &to_pt))
        continue;

      (*matches)[f.get()] = to_pt;

      // Add the new observations to the LocalMap.
      Vector2d frame_point(to_pt.x, to_pt.y);
      f->point->AddObservation({frame_point, to_view.frame});

      // Debugging.
      cv::Mat patch;
      cv::getRectSubPix(to_view.original, Size(kWindowSize, kWindowSize), to_pt, patch);
      int id = f->point->id();
      patches[id].push_front(patch.clone());
      if (patches[id].size() > 30)
        patches[id].pop_back();

      // TODO: Think about multiple matching here.
      break;
    }
  }
}

// Find points that are duplicates and stop tracking the most recent.
void CleanDuplicates(
    std::map<Feature*, Point2f>* matches
    ) {

  std::set<std::pair<int, int>> mask;
  for (auto& m : *matches) {
    auto p = make_pair(m.second.x/2, m.second.y/2);
    if (mask.count(p)) {
      m.first->point->set_flag(TrackedPoint::Flags::MISMATCHED);
      printf("Removed duplicated point %3d\n", m.first->point->id());
    } else {
      mask.insert(p);
    }
  }
}

// BuildView.
// MatchView.
//   do slam in here.
// MatchView.
// AddKeyFrame (which may add new features).

// Track features from the previous two frames.
// (There may be an assumption that the previous frames are from alternating
// cameras).
//
// Adds matched features to the localmap.
bool Matcher::Track(const Mat& img, Frame* frame, int camera, LocalMap* map, std::function<bool ()> update_frames) {
  auto& d = *data_;

  FeatureTracker tracker(Size(kWindowSize, kWindowSize));

  CHECK_NE(img.size().width, 0);
  CHECK_NE(img.size().height, 0);
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(frame);

  // Build an image pyamid for the new image.
  Mat grey;
  cvtColor(img, grey, CV_RGB2GRAY);

  View* view = new View;
  view->frame = frame;
  view->pyramid = tracker.MakePyramid(img, 6);
  view->original = img.clone();

  // For each live tracked point, run over previous matches attempting to
  // propogate forward into this view. For a given tracked point, if there
  // are multiple matches, we take the best.
  //
  // We occasionally remove views and all the associated matches.
 
  // Remove bad matches.
  for (auto& f : d.features) {
    if (!f->point->feature_usable())
      d.features.erase(f);
  }
 
  // matches propogated forward into the current view.
  std::map<Feature*, Point2f> matches;

  FindMatches(d.features, *view, false, &tracker, &matches);

  if (1 || matches.size() < 40) {
    int before = matches.size();
    if (update_frames != nullptr) {
      if (update_frames()) {
        // Inferred position of Frame* has been updated, so try again for matching points.
        FindMatches(d.features, *view, true, &tracker, &matches);
      }
    }
    printf("Started with %d, grew to %d after additional matching\n", before, (int) matches.size());
  }

  //CleanDuplicates(&matches);

  // If there are insufficient trackable features, add new features from the
  // current image that aren't too close to an existing feature. Pass in a
  // lambda to generate feature IDs.
  // TODO: Lift out constant.
  if (matches.size() >= 40)
    return true;

  // New keyframe, so add the matches permanantly.
  for (auto& m : matches) {
    Feature* f = m.first;
    const auto& pt = m.second;
    f->matches[view] = pt;
  }

  d.views.push_back(unique_ptr<View>(view));

  // Possibly add new features.
  vector<Point2f> added;
  AddNewFeatures(grey, matches, &added);
  printf("Adding new keyframe for camera %d on frame %d (added %d)\n", camera, frame->id(), (int)added.size());

  for (auto& pt : added) {
    Vector2d frame_point(pt.x, pt.y);

    // Add a new TrackedPoint to the local map.
    // Use Frame::Unproject to initialize the world space location of the tracked point.
    Feature* f = new Feature;
    auto location = view->frame->Unproject(frame_point / 530., 1500);
    f->point = map->AddPoint(d.next_fid++, location);
    f->point->AddObservation({frame_point, view->frame});

    f->matches[view] = pt;
    d.features.insert(unique_ptr<Feature>(f));

    // Debugging.
    cv::Mat patch;
    cv::getRectSubPix(view->original, Size(kWindowSize, kWindowSize), pt, patch);
    int id = f->point->id();
    patches[id].push_front(patch.clone());
    if (patches[id].size() > 30)
      patches[id].pop_back();
  }

  // Potentially remove an old view.
  if (d.views.size() > 4) {
    View* view = d.views.front().get();
    for (auto& f : d.features)
      f->matches.erase(view);
    d.views.pop_front();
  }

  return true;
}
