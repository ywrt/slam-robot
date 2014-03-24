#ifndef MATCHER_H_
#define MATCHER_H_

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

class LocalMap;

class Matcher {
 public:
  Matcher();
  ~Matcher();

  struct Data;

  bool Track(const cv::Mat& img, int camera, LocalMap* map);

 private:
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;

  std::unique_ptr<cv::Mat> descriptors_;

  std::vector<Data> data_;
};


#endif
