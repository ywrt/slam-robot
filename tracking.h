/*
 * tracking.h
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#ifndef TRACKING_H_
#define TRACKING_H_

#include <memory>
#include <map>
#include <stdint.h>

class Pose;
class LocalMap;
class OctaveSet;
class ImageData;
namespace cv {
class Mat;
}

struct  Tracking {
  Tracking();
  ~Tracking();

  int UpdateCorners(LocalMap* map, const ImageData& fdata, int frame_idx, vector<Vector2d>* tracked);
  void FindNewCorners(LocalMap* map, int frame_idx);
  int ProcessFrame(const cv::Mat& image, LocalMap* map);

  static const int kSearchFrames = 2;

  std::map<int, std::unique_ptr<ImageData>> data_;
};

#endif /* TRACKING_H_ */
