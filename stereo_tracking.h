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
class Framepair;

struct  StereoTracking {
  StereoTracking();

  int ProcessFrame(int width, int height, const uint8_t* left, const uint8_t* right, LocalMap* map);

  static const int kSearchFrames = 2;
  std::deque<std::unique_ptr<FramePair>> frames;
};

#endif /* TRACKING_H_ */
