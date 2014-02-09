/*
 * tracking.h
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#ifndef TRACKING_H_
#define TRACKING_H_

#include <stdint.h>

class Pose;
class LocalMap;
class OctaveSet;

struct  Tracking {
  Tracking();
  int UpdateCorners(LocalMap* map, int frame_num);
  void FindNewCorners(LocalMap* map, int frame_num);
  int ProcessFrame(uint8_t* data, int width, int height, LocalMap* map);

  void flip();

  static const int kSearchFrames = 2;
  OctaveSet* curr;
  OctaveSet* prev[kSearchFrames];
};

#endif /* TRACKING_H_ */
