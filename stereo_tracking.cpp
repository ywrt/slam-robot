/*
 * tracking.cpp
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <glog/logging.h>

#include "localmap.h"
#include "octaveset.h"
#include "slam.h"
#include "corners.h"

#include "stereo_tracking.h"

struct StereoFrame {
  pair<Octave> images;
  pair<CornerList, CornerList> corners;
  vector<TrackedPoint*> points;
};

StereoTracking::StereoTracking() { }

namespace {
// Search in the region 'r' in 'img2', searching for the corner that best
// matches the corners 'c' in img1.
int FindBestCorner(
    const Octave& img1,
    const Octave& img2,
    const CornerList& corners,
    const Pos& c,
    const Region& r) {

  Patch patch(img1.GetPatch(img1.space().convert(c)));
  int best_score = 1<<30;
  int best_idx = -1;
  for (int idx = corners.find(r); idx >= 0; idx = corners.next(r, idx)) {
    const Pos& p(corners.corners[idx]);
    int score = img2.Score(patch, p); // int score; fpos = o1.SearchPosition(o1.space().convert(p), patch, 3, &score);
    if (score < best_score) {
      best_score = score;
      best_idx = idx;
    }
  }
  if (best_idx < 0)
    return -1;
  if (best_score > 4000)
    return -1;
  //printf("score %d\n"  , best_score);
  return best_idx;
}

// Given a stereo pair of images, find points that occur in both images.
pair<CornerList, CornerList> FindMatchingPoints(const Octave& left, const Octave& right) {
  auto left_corners = FindCorners(left, 20, 15);
  auto right_corners = FindCorners(right, 18, 15);

  pair<CornerList, CornerList> result;
  for (auto&c : corners.corners) {
    int idx = FindBestCorner(left, right, right_corners, c, Region(Pos(0, c.y - 2), Pos(c.x + 2, c.y + 2));
    if (idx < 0)
      continue;
    const Pos& p = right_corners.corners[idx];

    // Now search in reverse to check we ended up where we started.
    int rev_idx = FindBestCorner(right, left, left_corners, p, Region(Pos(c.x - 2, c.y - 2), Pos(1e6, c.y + 2));
    if (rev_idx < 0)
      continue;

    const Pos& c1 = left_corners.corners[rev_idx];
    if (c1 != c) {
      // Failed to find the same point we started with.
      continue;
    }

    result.first.corners.push_back(c);
    result.second.corners.push_back(p);
  }
}

}  // namespace

int SteroTracking::ProcessFrames(int width, int height, const uint8_t* left, const uint8_t* right, LocalMap* map) {
  int frame_num = map->AddFrame();

  frames.push_front({});
  auto& now = frames[0];
  now.images.first.FillOctaves(left, width, height);
  now.images.second.FillOctaves(right, width, height);
  now.corners = FindMatchingPoints(now.images.first, now.images.second);
  now.points.resize(now.corners.size());

  // Find corners that occur in previous frames and either update or
  // add them to the LocalMap.
  for (int i = 0; i < now.corners.first.size(); ++i) {
    const auto& left_c = now.corners.first.corners[i];
    const auto& right_c = now.corners.second.corners[i];
    for (int f = 1 ; f < kSearchFrames; ++f) {
      int idx1 = FindBestCorner(now.images.first, frames[f].images.first, frames[f].corners.first, left_c, Region(left_c - 30, left_c + 30));
      if (idx1 < 0) continue;  // Didn't match any corner.
      int idx2 = FindBestCorner(now.images.second, frames[f].images.second, frames[f].corners.second, right_c, Region(right_c - 30, right_c + 30));
      if (idx1 != idx2) continue;  // Didn't match the same point on left and right.

      // Now what? We know that corner 'idx1' in 'frames[f]' matches corner 'i' in 'now'.
      // If it's not previously been tracked, then start tracking it. Else add it as
      // an observation to the tracked point.
      TrackedPoint* pt = NULL;
      if (frames[i].points[idx1]) {
        pt = frames[i].points[idx1];
      } else {
        // Try and find it via descriptor matching.
        pt = FindTrackedPointByDesc(now.images.left, left_c);
        if (!pt) {
          pt = new TrackedPoint;
          pt->AddObservation(
              frame_num - f,
              frames[f].corners.first.corners[idx1],
              frames[f].corners.second.corners[idx1]);
          AddTrackedPointByDesc(pt, frames[f].images.left, frames[f].corners.first.corners[idx1]);
        }
      }
      now.points[i] = pt;
      pt->AddObservation(frame_num, left_c, right_c);
      
      break; // Found a matching point, all done.
    }
  }

  if (frames.size() > kSearchFrames) {
    frames.pop_back();
  }
  return frame_num;
}
