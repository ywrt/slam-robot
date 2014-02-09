/*
 * tracking.h
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#ifndef TRACKING_H_
#define TRACKING_H_

#include <stdint.h>

class LocalMap;
class OctaveSet;

struct  StereoTracking {
  StereoTracking();

  int ProcessFrame(int width, int height, const uint8_t* left, const uint8_t* right, LocalMap* map);

  static const int kSearchFrames = 2;
  OctaveSet* curr;
  OctaveSet* prev[kSearchFrames];
};

#endif /* TRACKING_H_ */
