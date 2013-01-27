/*
 * tracking.h
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#ifndef TRACKING_H_
#define TRACKING_H_

#include <stdint.h>
#include <eigen3/Eigen/Eigen>

class Frame;
class LocalMap;
class OctaveSet;

struct  Tracking {
  Tracking();
  void ComputeHomography(const Frame& f1,
                         const Frame& f2,
                         Eigen::Matrix3d* homog);
  Vector2d ComputePoint(
      const Eigen::Vector2d& point,
      const Eigen::Matrix3d& homog);
  int UpdateCorners(LocalMap* map, int frame_num);
  void FindNewCorners(LocalMap* map, int frame_num);
  int ProcessFrame(uint8_t* data, int width, int height, LocalMap* map);

  void flip();

  static const int kSearchFrames = 2;
  OctaveSet* curr;
  OctaveSet* prev[kSearchFrames];
};

#endif /* TRACKING_H_ */
