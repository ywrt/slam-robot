#ifndef Hessian_H_
#define Hessian_H_

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

extern int debug;

class HessianTracker {
  public:
    HessianTracker(cv::Size size) : size_(size), len_(size.width * size.height) {
      mask_.resize(len_);
      for (int y = 0; y < size.width; ++y)  {
        for (int x = 0; x < size.width; ++x)  {
          double rx = (0.5 * size.width - x);
          double ry = (0.5 * size.height- y);
          double rr = rx * rx + ry * ry;
          mask_[y * size.width + x] = 1. / (15. + rr);
        }
      }

      // Normalize.
      double sum(0);
      for (const auto& v : mask_)
        sum += v;
      double scale = len_ / sum;
      for (auto& v : mask_)
        v *= scale;
      printf("scale %f\n", scale);
    }

  struct Patch {
    Patch(cv::Size s) :
       size(s), data(s.width * s.height) {}

    cv::Size size;
    vector<float> data;
    float mean;
    float sumsq;
  };

  struct GradImage {
    cv::Mat image;
  };

  typedef vector<GradImage> Pyramid;

  enum Status {
    OK,
    SMALL_DET,
    OUT_OF_BOUNDS,
  };

  Patch GetPatch(const GradImage& gimg, const cv::Point2f& pt) {
    Patch patch(size_);
    CHECK_EQ(CV_32F, gimg.image.type());

    CHECK_EQ(len_, patch.data.size());
    for (int i = 0; i < len_; ++i) {
      patch.data[i] = 0.;
    }

    cv::Point2f p = pt;
    cv::Rect r(0, 0, size_.width, size_.height);
    if (p.x < 0.5 * size_.width) {
      int d = (0.5 * size_.width - p.x) + 0.9999;
      p.x += 0.5 * d;
      r = cv::Rect(d, r.y, r.width - d, r.height);
    }

    if (p.y < 0.5 * size_.height) {
      int d = (0.5 * size_.height - p.y);
      p.y += 0.5 * d;
      r = cv::Rect(r.x, d, r.width, r.height - d);
    }

    cv::Mat m(r.size(), CV_32F, &patch.data[r.x + r.y * size_.width], size_.width * sizeof(float)); 

    cv::getRectSubPix(
        gimg.image,
        m.size(),
        p,
        m);

    float sum(0), sum_sq(0);
    for (float d : patch.data) {
      sum += d;
      sum_sq += d * d;
    }
    patch.mean = sum / len_;
    patch.sumsq = sum_sq / len_;
    return patch;
  }

  vector<GradImage> MakePyramid(const cv::Mat& img, int depth) {
    vector<GradImage> result(depth);

    GradImage g;
    cv::Mat grey;
    cvtColor(img, grey, CV_RGB2GRAY);
    grey.convertTo(g.image, CV_32F, 1./255.);
    cv::GaussianBlur(g.image, g.image, cv::Size(5, 5), 1.1, 1.1);

    result[0] = g;
    for (int i = 1 ; i < depth; ++i) {
      GradImage& curr = result[i];
      GradImage& prev = result[i - 1];
      cv::Size s((prev.image.cols + 1) / 2, (prev.image.rows + 1) / 2);

      curr.image = cv::Mat(s, CV_32F);

      cv::pyrDown(prev.image, curr.image);
      cv::GaussianBlur(curr.image, curr.image, cv::Size(5, 5), .8, .8);

#if 0
      cv::resize(prev.image, curr.image, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
      cv::resize(prev.gradx, curr.gradx, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
      cv::resize(prev.grady, curr.grady, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
      curr.gradx *= 2.f;
      curr.grady *= 2.f;
#endif
    }

    CHECK_EQ(CV_32F, result[0].image.type());
    return result;
  }

  // Lighting invariant difference between patches.
  float ScorePatchMatch(const Patch& p1, const Patch& p2) {
    float sum = 0;
    float alpha = sqrt(p1.sumsq / p2.sumsq);
    float beta = p1.mean - alpha * p2.mean;
    for (int i = 0; i < len_; ++i) {
      if (p1.data[i] == 0 || p2.data[i] == 0)
        continue;
      float diff = p1.data[i] - p2.data[i] * alpha - beta;
      diff = diff * diff;
      sum += diff * mask_[i];
    }
    return sum;
  }

  // Compute the gradient vector and hessian matrix by numeric approximation.
  //
  // TODO: Make this faster. Use the fact that it's doing interpolation
  // of points anyway to directly compute the gradients.
  float BruteHessian(const GradImage& g, const Patch& patch, cv::Point2f pt,
      float* dx,
      float* dy,
      float* dxx,
      float* dxy,
      float* dyx,
      float* dyy) {
    const double h = 0.02;
    double sad0 = ScorePatchMatch(patch, GetPatch(g, pt));
    double sadn1x = ScorePatchMatch(patch, GetPatch(g, cv::Point2f(pt.x - h, pt.y)));
    double sadn1y = ScorePatchMatch(patch, GetPatch(g, cv::Point2f(pt.x, pt.y - h)));

    double sadp1x = ScorePatchMatch(patch, GetPatch(g, cv::Point2f(pt.x + h, pt.y)));
    double sadp1y = ScorePatchMatch(patch, GetPatch(g, cv::Point2f(pt.x, pt.y + h)));
    double sadxy = ScorePatchMatch(patch, GetPatch(g, cv::Point2f(pt.x + h, pt.y + h)));

    *dx = 0.5 * (sadp1x - sadn1x) / h;
    *dy = 0.5 * (sadp1y - sadn1y) / h;

    *dxx = ((sadp1x - sad0) / h - (sad0 - sadn1x) / h) / h;
    *dyy = ((sadp1y - sad0) / h - (sad0 - sadn1y) / h) / h;
    *dxy = ((sadxy - sadp1y) / h - (sadp1x - sad0) / h) / h;
    *dyx = ((sadxy - sadp1x) / h - (sadp1y - sad0) / h) / h;

    return sad0;
  }

  // Get a patch stack.
  vector<Patch> GetPatches(const vector<GradImage>& stack, cv::Point2f pt) {
    vector<Patch> result;
    for (unsigned int i = 0; i < stack.size(); ++i) {
      result.push_back(GetPatch(stack[i], pt));
      pt *= 0.5;
    }
    return result;
  }

  Status Track(
      const GradImage& gimg,
      const Patch& patch,
      float threshold,
      int max_iterations,
      cv::Point2f* pt) {

    float x = pt->x;
    float y = pt->y;
    if (debug) printf("   Track\n");

    const float margin = 0.01;
    cv::Size s = gimg.image.size();
    for (int iterations = 0; iterations < max_iterations; ++iterations) {
      if (x < margin || y < margin || (x + margin) > s.width || (y + margin) > s.height) {
        if (debug) printf("out of bounds %7.2f, %7.2f\n", x, y);
        pt->x = x;
        pt->y = y;
        return OUT_OF_BOUNDS;
      }

      float mdx(-1), mdy(-1), mdxx(-1), mdxy(-1), mdyx(-1), mdyy(-1);
      float sad = BruteHessian(gimg, patch, cv::Point2f(x, y), &mdx, &mdy, &mdxx, &mdxy, &mdyx, &mdyy);

      Eigen::Matrix2d H;
      H << mdxx, mdxy, mdyx, mdyy;
      Eigen::Vector2d g;
      g << mdx, mdy;
      //std::cout << H << "\n";

      Eigen::Vector2d jj = H.inverse() * g;

      if (debug)
      printf(" * [%7.2f, %7.2f] + [%7.2f, %7.2f] J [%7.2f, %7.2f] sad %6.4f\n",
          x, y, -jj(0), -jj(1), g[0], g[1], sad);

      float dx = -jj(0);
      float dy = -jj(1);

      if ((dx*dx + dy*dy) > 1) {
        dx /= sqrt(dx * dx + dy * dy);
        dy /= sqrt(dx * dx + dy * dy);
      }

      // Update current point.
      x += max(-1.f, min(1.f, dx));
      y += max(-1.f, min(1.f, dy));
      if (fabs(dx) < threshold && fabs(dy) < threshold)
        break;
    }

    if (debug) printf(" * [%7.2f, %7.2f] => [%7.2f, %7.2f]\n", pt->x, pt->y, x, y);

    pt->x = x;
    pt->y = y;
    return OK;
  }

  Status TrackFeature(
      const vector<GradImage>& stack,
      const vector<Patch>& patches,
      float threshold,
      int max_iterations,
      cv::Point2f* pt) {
    int lvls = stack.size();
    if (debug) printf("Tracking %7.2f, %7.2f\n", pt->x, pt->y);
    cv::Point2f p = (*pt) * (1./ (1 << (lvls - 1)));
    for (int i = lvls - 1 ; i > 0; --i) {
      Status status = Track(stack[i], patches[i], threshold, max_iterations, &p);
      if (status != OK)
        return status;
      p *= 2.;  // Upscale for the next larger image.
    }
    Status status = Track(stack[0], patches[0], threshold, max_iterations, &p);
    if (status != OK)
      return status;

    *pt = p;
    return OK;
  }

 private:
  cv::Size size_;
  int len_;
  vector<float> mask_;
};

#endif
