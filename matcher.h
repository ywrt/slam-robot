#ifndef MATCHER_H_
#define MATCHER_H_

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

class LocalMap;
class Frame;

class Matcher {
 public:
  Matcher();
  ~Matcher();

  struct Data;

  bool Track(const cv::Mat& img, Frame* frame, LocalMap* map);

 private:
  std::unique_ptr<Data> data_;
};


#endif
