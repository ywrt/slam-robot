/*
 * tracking.cpp
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */
#include <map>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <glog/logging.h>

#include "localmap.h"
#include "octaveset.h"
#include "slam.h"
#include "corners.h"

#include "stereo_tracking.h"

struct FrameCorners {
  Octave image;
  CornerList corners;
};

struct FramePair {
  FrameCorners left;
  FrameCorners right;
  map<Pos, Pos> lr_matches; // left corners that match right corners.
  map<Pos, Pos> ll_matches; // left corners that match the previous left frame.
  map<Pos, Pos> rr_matches; // right corners that match the previous right frame.
  map<Pos, TrackedPoint*> points;  // map from left corners to tracked points.
};

StereoTracking::StereoTracking() { }
StereoTracking::~StereoTracking() { }

namespace {
// Search in the region 'r' in 'img2', searching for the corner that best
// matches the corners 'c' in img1. Only return corners where the disparity
// is less than 'max_disparity'.
Pos FindBestCorner(
    const FrameCorners& img1,
    const FrameCorners& img2,
    const Pos& c,
    const Region& r,
    const int max_disparity) {

  Patch patch(img1.image.GetPatch(img1.image.space().convert(c)));
  int best_score = 1<<30;
  Pos best_pos(Pos::invalid());
  for (int idx = img2.corners.find(r); idx >= 0; idx = img2.corners.next(r, idx)) {
    const Pos& p(img2.corners.corners[idx].pos);
    int score = img2.image.Score(patch, p); // int score; fpos = o1.SearchPosition(o1.space().convert(p), patch, 3, &score);
    if (score < best_score) {
      best_score = score;
      best_pos = p;
    }
  }
  if (best_pos == Pos::invalid()) {
    LOG(INFO) << c << " had no corners in region";
    return Pos::invalid();
  }
  if (best_score > max_disparity) {
    LOG(INFO) << c << " had crazy score: " << best_score << " for " << best_pos;
    return Pos::invalid();
  }
  //printf("score %d\n"  , best_score);
  return best_pos;
}

// Search in the region 'r' in 'img2', searching for the corner that best
// matches the corners 'c' in img1. Only return corners where the disparity
// is less than 'max_disparity'.
Pos FindBestCornerDense(
    const FrameCorners& img1,
    const FrameCorners& img2,
    const Pos& c,
    const Region& r,
    const int max_disparity) {

  Patch patch(img1.image.GetPatch(img1.image.space().convert(c)));
  int best_score = 1<<30;
  Pos best_pos;
  Region mask = img2.image.clipped_region(r, Patch::kPatchRadius);
  LOG(INFO) << "Searching " << mask.ll << " to " << mask.ur << " for " << c;
  for (const auto& p : mask) {
    int score = img2.image.Score(patch, p);
    if (score < best_score) {
      best_score = score;
      best_pos = p;
    }
  }
  if (best_score > max_disparity) {
    LOG(INFO) << c << " had crazy score: " << best_score << " for " << best_pos;
    return Pos::invalid();
  }
  //printf("score %d\n"  , best_score);
  return best_pos;
}

// Given a pair of images, find points that occur in both images.
// Returns a list of pair of indexes of matching corners.
map<Pos, Pos> FindMatchingPoints(const FrameCorners& left, const FrameCorners& right, const Region& r, int max_disparity) {
  map<Pos, Pos> result;
  for (auto&c : left.corners.corners) {
    if (c.score < 20) continue;  // Only use somewhat better corners as a starting point for improved repeatability.
    const Pos& lpos = c.pos;
    Pos right_pos = FindBestCornerDense(left, right, lpos, Region(r.ll + lpos, r.ur + lpos), max_disparity);
    if (right_pos == Pos::invalid())
      continue;

    // Now search in reverse to check we ended up where we started.
    Pos left_pos = FindBestCornerDense(right, left, right_pos, Region(right_pos - r.ur, right_pos - r.ll), max_disparity);
    if (left_pos == Pos::invalid())
      continue;

    if (left_pos != lpos) {
      // Failed to find the same point we started with.
      LOG(INFO) << "Mis-match: " << left_pos << " versus " << lpos;
      continue;
    }
    result[left_pos] = right_pos;
    LOG(INFO) << "Left " << left_pos << " matches " << right_pos << "\n";
  }
  return result;
}

}  // namespace


// Process a stereo pair of frames.
// We match left to right, left to the previous left, and right to the previous right.
int StereoTracking::ProcessFrames(int width, int height, const uint8_t* left, const uint8_t* right, LocalMap* map) {
  LOG(INFO) << "ProcessFrames";
  const int max_disparity = 12000;

  int left_num = map->AddFrame();
  int right_num = map->AddFrame();

  std::unique_ptr<FramePair> fp(new FramePair);
  fp->left.image = Octave(left, width, height);
  fp->right.image = Octave(right, width, height);
  fp->left.corners = FindCorners(fp->left.image, 18, 15);  // min score 18, non-max-supression radius 15.
  fp->right.corners = FindCorners(fp->right.image, 18, 15);
  // Find left-right matches.
  fp->lr_matches = FindMatchingPoints(fp->left, fp->right, Region(Pos(-180,-15), Pos(15, 15)), max_disparity);
  // Find matching points between the left and the previous left image.
  if (frames.size()) {
    LOG(INFO) << "Has previous";
    fp->ll_matches = FindMatchingPoints(fp->left, frames.front()->left, Region(Pos(-20,-20),Pos(20,20)), max_disparity);
    fp->rr_matches = FindMatchingPoints(fp->right, frames.front()->right, Region(Pos(-20,-20),Pos(20,20)), max_disparity);
  }

  // Find left-right-corners that occured in previous frames and either update or
  // add them to the LocalMap.
  for (const auto& match : fp->lr_matches) {
    int ll_match = fp->ll_matches.count(match.first);
    int rr_match = fp->rr_matches.count(match.second);
    TrackedPoint* pt = NULL;
    if (ll_match && rr_match) {
      // Full match. LR, LL and RR. Retrieve the tracked point.
      pt = frames[0]->points[fp->ll_matches[match.first]];
      // TODO: Handle non-full matches. (i.e. LR+LL, or LR+RR only).
    } else if (!ll_match && !rr_match) {
      // TODO: Try and find it via descriptor matching.
      // pt = FindTrackedPointByDesc(now.images.left, left_c);

      // New point. Start tracking it.
      if (!pt) {
        pt = map->AddPoint();
      }
    }
    if (!pt) continue;

    auto left_vec = fposToVector(fp->left.image.space().convert(match.first));
    auto right_vec = fposToVector(fp->right.image.space().convert(match.second));
    
    pt->AddObservation(Observation(left_vec, left_num, 0));
    pt->AddObservation(Observation(right_vec, right_num, 1));
  }

  frames.push_front(std::move(fp));
  if (frames.size() > kSearchFrames) {
    frames.pop_back();
  }
  return right_num;
}

const CornerList& StereoTracking::left_corners() {
  return frames.front()->left.corners;
}

const CornerList& StereoTracking::right_corners() {
  return frames.front()->right.corners;
}
