#ifndef VIDEO_H_
#define VIDEO_H_

#include <stdio.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

// Interface: A source of images. Typically a camera or
// a video file.
class ImageSource {
 protected:
  ImageSource() {}
 public:
  virtual ~ImageSource() {}
  virtual bool GetObservation(int camera, int frame_id, cv::Mat* img) = 0;
  virtual bool Init() = 0;
};

// Source images from a single video file.
class ImageSourceFiles : public ImageSource {
 public:
  ImageSourceFiles(const std::string& dir) : dir_(dir) {}
  bool Init() { return true; }
  virtual bool GetObservation(int, int frame_id, cv::Mat* img) {
    char b[100];
    sprintf(b, "%08d.png", frame_id);
    *img = cv::imread(dir_ + "/" + b);

    return img->data != NULL;
  }

 private:
  const std::string dir_;
};

// Source images from a single video file.
class ImageSourceMono : public ImageSource {
 public:
  ImageSourceMono(const char* filename) : filename_(filename), cam_(filename) {}
  ImageSourceMono(int cam) : filename_("Camera"), cam_(cam) {}
  bool Init() {
    if (!cam_.isOpened()) {
      printf("Failed to open video file: %s\n", filename_);
      return false;
    }
    cam_.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cam_.set(CV_CAP_PROP_FPS, 10);
    return true;
  }

  virtual bool GetObservation(int, int, cv::Mat* img) {
    return cam_.read(*img);
  }

 private:
  const char* filename_;
  cv::VideoCapture cam_;
};

// Source images alternately from two video files.
class ImageSourceDuo : public ImageSource {
 public:
  ImageSourceDuo(ImageSource* src1, ImageSource* src2) :
      src1_(src1), src2_(src2) {}


  bool Init() {
    return src1_->Init() && src2_->Init();
  }

  virtual bool GetObservation(int camera, int id, cv::Mat* img) {
    if (camera == 0) {
      return src1_->GetObservation(camera, id, img);
    } else {
      return src2_->GetObservation(camera, id, img);
    }
  }

 private:
  std::unique_ptr<ImageSource> src1_;
  std::unique_ptr<ImageSource> src2_;
};

// Source images from a V4L2 source
class VideoDev : public ImageSource {
 public:
  VideoDev(const char* device, int num_buf) :
      device_(device), fd_(-1), num_buf_(num_buf) {}
  ~VideoDev();

  bool Init();
  virtual bool GetObservation(int, int frame_id, cv::Mat* img);

 private:
  struct Buffer;

  const char* device_;
  int fd_;
  int num_buf_;
  std::vector<Buffer*> buffers_;
};

#endif  // VIDEO_H_
