/*
 * tracking.h
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#ifndef TRACKING_H_
#define TRACKING_H_

#include <deque>
#include <memory>
#include <stdint.h>

class LocalMap;
class OctaveSet;
struct FramePair;
class CornerList;

struct  StereoTracking {
  StereoTracking();
  ~StereoTracking();

  int ProcessFrames(int width, int height, const uint8_t* left, const uint8_t* right, LocalMap* map);

  const CornerList& left_corners();
  const CornerList& right_corners();

  static const int kSearchFrames = 2;
  std::deque<std::unique_ptr<FramePair>> frames;
};

#endif /* TRACKING_H_ */
