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

class Feature {
  Vector2d pt;
  int id;
}

struct  Tracking {
  Tracking();
  ~Tracking();

  int UpdateCorners(LocalMap* map, const ImageData& fdata, int frame_idx, vector<Vector2d>* tracked);
  void FindNewCorners(LocalMap* map, int frame_idx);
  const std::vector<Feature>& ProcessFrame(const cv::Mat& image);

  std::vector<Feature> features1_;
  std::vector<Feature> features2_;

  std::unique_ptr<ImageData> image1_;
  std::unique_ptr<ImageData> image2_;
  int last_id;
};

#endif /* TRACKING_H_ */
