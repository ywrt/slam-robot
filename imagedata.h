#ifndef IMAGEDATA_H_
#define IMAGEDATA_H_

#include <vector>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

struct Views {
  cv::Mat image;
  cv::Mat smoothed;
  cv::Mat grey;
};


class ImageData {
 public:
  ImageData();
  ImageData(const cv::Mat& img);
  ~ImageData();

  bool UpdateCorner(const ImageData& prev, const Eigen::Matrix3d& homograpy, int min_score, Eigen::Vector2d* pt) const;

 private:
  std::vector<Views> octaves_;
};

#endif
