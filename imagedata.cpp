#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

#include "imagedata.h"


ImageData::ImageData() { }
ImageData::~ImageData() { }

ImageData::ImageData(const cv::Mat& image) : octaves_(4) {
  const cv::Mat* prev = &image;
  double scale = 1;
  for (auto& o : octaves_) {
    // Rescale from previous.
    cv::resize(*prev, o.image, cv::Size(), scale, scale, INTER_AREA);
    // Smooth it.
    cv::GaussianBlur(o.image, o.smooth, cv::Size(), 1.5, 1.5, cv::BORDER_CONSTANT);

    scale = 0.5;
    prev = &o.image;
  }
}

namespace {

// Map a point from [-1,1]x[-1,1] + (dx,dy) to [0,1]x[0,1] via the homography.
//( homography * ( [-1,1]*[-1,1] + (dx,dy) ) + 1 ) * 0.5
Vector2d Transform(const Matrix3d homography, const Vector2d& pt, double dx, double dy) {
  Vector3d hpt;
  hpt(0) = pt(0) + dx;
  hpt(1) = pt(1) + dy;
  hpt(2) = 1.;

  hpt = homography * hpt;

  Vector2d result;
  result(0) = (hpt(0) + 1) * 0.5) / hpt(2);
  result(1) = (hpt(1) + 1) * 0.5) / hpt(2);

  return result;
}

bool Outside(const Vector2d& size, const Vector2d& pt) {
  if (pt(0) < 0 || pt(1) < 0) return true;
  if (pt(0) > size(0) || pt(1) > size(1)) return true;
  return false;
}

// Extract a 9x9 patch from 'src' as transformed by the inverse homography.
bool Extract(const cv::Mat& src, const Eigen::Matrix3d& homography, const Vector2d& pt, cv::Mat* patch) {
  Vector2d size;
  size(0) = src.size().width();
  size(1) = src.size().height();

  double dx = 4. / size.width();
  double dy = 4. / size.height();

  Vector2d ul = Transform(homography, pt, -dx, -dy).array() * size;
  Vector2d ur = Transform(homography, pt, dx, -dy).array() * size;
  Vector2d ll = Transform(homography, pt, -dx, dy).array() * size;
  Vector2d lr = Transform(homography, pt, dx, dy).array() * size;

  Point2f c_src[4];
  Point2f c_dst[4];

  c_src[0].x = ul(0); c_src[0].y = ul(1);
  c_src[1].x = ur(0); c_src[1].y = ur(1);
  c_src[2].x = ll(0); c_src[2].y = ll(1);
  c_src[3].x = lr(0); c_src[3].y = lr(1);

  c_dst[0].x = 0; c_dst[0].y = 0;
  c_dst[1].x = 8; c_dst[1].y = 0;
  c_dst[2].x = 0; c_dst[2].y = 8;
  c_dst[3].x = 8; c_dst[3].y = 8;

  Mat m = cv::getPerspectiveTransform(c_src, c_dst);

  *patch = Mat(9, 9, src.type());
  warpPerspective(src, *patch, m, Size(9, 9));

  return true;
}

int Score(const cv::Mat& image, const cv::Mat& patch, double x, double y) {
  int ix = 0.5 + x - 4;
  int iy = 0.5 + y - 4;

  int sum = 0;
  for (int dy = 0; dy < 9; ++dy) {
    for (int dx = 0; dx < 9; ++dx) {
      auto v1 = patch.at<cv::Vec3b>(dx, dy);
      auto v2 = image.at<cv::Vec3b>(x + dx, y + dy);
      sum += abs(v1.val[0] - v2.val[0]);
      sum += abs(v1.val[1] - v2.val[1]);
      sum += abs(v1.val[2] - v2.val[2]);
    }
  }
  return sum;
}

}  // namespace


// Refine a corner.
bool ImageData::UpdateCorner(
    const ImageData& prev,
    const Eigen::Matrix3d& homograpy,
    int min_score,
    Eigen::Vector2d* pt) const {
  // 'pt' is in destination space: [-1,1]^2.
  // 'homography' maps from dst to src.
  cv::Mat M(3, 3, CV_64F, homograpy.data());

  for (int i = 0; i < octaves_.size(); ++i) {
    const Views& src = ref.octaves_[i];
    const Views& dst = octaves_[i];

    // Extract a warped patch from src.
    cv::Mat patch;
    if (!Extract(src.smooth, homography, pt, &patch))  // Get the 9x9 patch.
      return false;  // patch is outside source image.

    // Convert Vector2d to image location.
    Vector2d p;
    p(0) = (pt->(0) + 1) * 0.5 * dst.size().width();
    p(1) = (pt->(1) + 1) * 0.5 * dst.size().height();

    // Return false if too close to edge to match patch.
    if (p(0) < 5 || p(1) < 5)
      return false;
    if ((p(0) + 5) > dst.smoothed.size().width() || ((p(1) + 5) > dst.smoothed.size().height()))
      return false;

    // Search in dst for best match.
    int best_score = 1e9;
    Vector2d best_point = p;
    for (int dy = -2; dy <= 2 ; ++dy) {
      for (int dx = -2; dx <= 2 ; ++dx) {
        int score = Score(dst.smoothed, patch, p(0) + dx , p(1) + dy);
        if (score > best_score)
          continue;
        best_score = score;
        best_point << p(0) + dx, p(1) + dy;
      }
    }

    // abort if score is worse than min_score.
    if (best_score > min_score)
      return false;

    // Update point with refined match.
    pt->(0) = (best_point(0) / dst.size().width()) * 2 - 1;
    pt->(1) = (best_point(1) / dst.size().width()) * 2 - 1;
  }
  return true;
}
