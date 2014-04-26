#ifndef MATCHER_H_
#define MATCHER_H_

#include <memory>
#include <vector>
#include <functional>

#include <opencv2/opencv.hpp>

class LocalMap;
class Frame;

class Matcher {
 public:
  Matcher();
  ~Matcher();

  struct Data;

  bool Track(
      const cv::Mat& img,
      Frame* frame,
      int camera,
      LocalMap* map,
      std::function<bool ()> update_frames
      );

 private:
  std::unique_ptr<Data> data_;
};

const map<int, deque<cv::Mat>>& GetPatches();

#endif
