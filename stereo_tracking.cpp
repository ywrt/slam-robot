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
  map<int, int> lr_matches; // left corners that match right corners.
  map<int, int> ll_matches; // left corners that match the previous left frame.
  map<int, int> rr_matches; // right corners that match the previous right frame.
  map<int, TrackedPoint*> points;  // map from left corners to tracked points.
};

StereoTracking::StereoTracking() { }

namespace {
// Search in the region 'r' in 'img2', searching for the corner that best
// matches the corners 'c' in img1. Only return corners where the disparity
// is less than 'max_disparity'.
int FindBestCorner(
    const FrameCorners& img1,
    const FrameCorners& img2,
    const Pos& c,
    const Region& r,
    const int max_disparity) {

  Patch patch(img1.image.GetPatch(img1.image.space().convert(c)));
  int best_score = 1<<30;
  int best_idx = -1;
  for (int idx = img2.corners.find(r); idx >= 0; idx = img2.corners.next(r, idx)) {
    const Pos& p(img2.corners.corners[idx].pos);
    int score = img2.image.Score(patch, p); // int score; fpos = o1.SearchPosition(o1.space().convert(p), patch, 3, &score);
    if (score < best_score) {
      best_score = score;
      best_idx = idx;
    }
  }
  if (best_idx < 0)
    return -1;
  if (best_score > max_disparity)
    return -1;
  //printf("score %d\n"  , best_score);
  return best_idx;
}

// Given a pair of images, find points that occur in both images.
// Returns a list of pair of indexes of matching corners.
map<int, int> FindMatchingPoints(const FrameCorners& left, const FrameCorners& right, int max_disparity) {
  auto right_corners = FindCorners(right, 18, 15);

  map<int, int> result;
  for (auto&c : corners.corners) {
    if (c.score < 20) continue;  // Only use somewhat better corners as a starting point for improved repeatability.
    int right_idx = FindBestCorner(left, right, c, Region(Pos(0, c.y - 2), Pos(c.x + 2, c.y + 2), max_disparity);
    if (right_idx < 0)
      continue;
    const Pos& p = right_corners.corners[right_idx];

    // Now search in reverse to check we ended up where we started.
    int left_idx = FindBestCorner(right, left, p, Region(Pos(c.x - 2, c.y - 2), Pos(1e6, c.y + 2), max_disparity);
    if (left_idx < 0)
      continue;

    const Pos& c1 = left_corners.corners[left_idx];
    if (c1 != c) {
      // Failed to find the same point we started with.
      continue;
    }
    result[left_idx] = right_idx;
  }
}

}  // namespace


// Process a stereo pair of frames.
// We match left to right, left to the previous left, and right to the previous right.
int SteroTracking::ProcessFrames(int width, int height, const uint8_t* left, const uint8_t* right, LocalMap* map) {
  const int max_disparity = 4000;

  int left_num = map->AddFrame();
  int right_num = map->AddFrame();

  std::unique_ptr<FramePair> fp = new FramePair;
  fp->left.image = Octave(width, height, left);
  fp->right.image = Octave(width, height, right);
  fp->left.corners = FindCorners(fp->left.image, 18, 15);  // min score 18, non-max-supression radius 15.
  fp->right.corners = FindCorners(fp->right.image, 18, 15);
  // Find left-right matches.
  fp->lr_matches = FindMatchingPoints(fp->left, fp->right, max_disparity);
  // Find matching points between the left and the previous left image.
  if (frames.size()) {
    fp->ll_matches = FindMatchingPoints(fp->left, frames.head()->left, max_disparity);
    fp->rr_matches = FindMatchingPoints(fp->right, frames.head()->right, max_disparity);
  }

  // Find left-right-corners that occured in previous frames and either update or
  // add them to the LocalMap.
  for (const auto& match : fp->lr_matches) {
    bool ll_match = fp->ll_matches.count(match.first);
    bool rr_match = fp->rr_matches.count(match.second);
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
    const auto& left_pos = fp->left.corners[match.first].pos;
    const auto& right_pos = fp->right.corners[match.second].pos;

    pt->AddObservation(Observation(left_pos.x, left_pos.y, frame_num, 0));
    pt->AddObservation(Observation(right_pos.x, right_pos.y, frame_num, 1));
  }

  frames.push_front(fp);
  if (frames.size() > kSearchFrames) {
    frames.pop_back();
  }
  return frame_num;
}
