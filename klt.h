#ifndef KLT_H_
#define KLT_H_

#include <opencv2/opencv.hpp>

extern int debug;

class KLTTracker {
  public:
    KLTTracker(cv::Size size) : size_(size), len_(size.width * size.height) {}

  struct Patch {
    Patch(cv::Size s) :
       size(s), len(s.width * s.height), data(len), gradx(len), grady(len) {}

    cv::Size size;
    int len;
    vector<float> data;
    vector<float> gradx;
    vector<float> grady;
    float mean;
    float sumsq;
  };

  struct GradImage {
    cv::Mat image;
    cv::Mat gradx;
    cv::Mat grady;
  };

  enum Status {
    OK,
    SMALL_DET,
    OUT_OF_BOUNDS,
  };

  Patch GetPatch(const GradImage& gimg, const cv::Point2f& pt) {
    Patch patch(size_);
    CHECK_EQ(CV_32F, gimg.image.type());
    CHECK_EQ(CV_32F, gimg.gradx.type());
    CHECK_EQ(CV_32F, gimg.grady.type());

    CHECK_EQ(len_, patch.data.size());
    CHECK_EQ(len_, patch.gradx.size());
    CHECK_EQ(len_, patch.grady.size());
    for (int i = 0; i < len_; ++i) {
      patch.data[i] = patch.gradx[i] = patch.grady[i] = 0;
    }

    cv::getRectSubPix(
        gimg.image,
        size_,
        pt,
        cv::Mat(size_.width, size_.height, CV_32F, &patch.data[0], size_.width * sizeof(float))); 
    cv::getRectSubPix(
        gimg.gradx,
        size_,
        pt,
        cv::Mat(size_.width, size_.height, CV_32F, &patch.gradx[0], size_.width * sizeof(float))); 
    cv::getRectSubPix(
        gimg.grady,
        size_,
        pt,
        cv::Mat(size_.width, size_.height, CV_32F, &patch.grady[0], size_.width * sizeof(float))); 

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
    Sobel(g.image, g.gradx, CV_32F, 1, 0, CV_SCHARR, 1.);
    Sobel(g.image, g.grady, CV_32F, 0, 1, CV_SCHARR, 1.);

    result[0] = g;
    for (int i = 1 ; i < depth; ++i) {
      GradImage& curr = result[i];
      GradImage& prev = result[i - 1];
      cv::Size s((prev.image.cols + 1) / 2, (prev.image.rows + 1) / 2);

      curr.image = cv::Mat(s, CV_32F);
      curr.gradx = cv::Mat(s, CV_32F);
      curr.grady = cv::Mat(s, CV_32F);

      cv::pyrDown(prev.image, curr.image);
      Sobel(curr.image, curr.gradx, CV_32F, 1, 0, CV_SCHARR, 1.);
      Sobel(curr.image, curr.grady, CV_32F, 0, 1, CV_SCHARR, 1.);
      //cv::pyrDown(prev.gradx, curr.gradx);
      //cv::pyrDown(prev.grady, curr.grady);
    }

    CHECK_EQ(CV_32F, result[0].image.type());
    return result;
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
      if (debug)
    printf("   Track\n");

    cv::Size s = gimg.image.size();
    double odx(0), ody(0);
    bool xvalid = false;
    bool yvalid = false;
    for (int iterations = 0; iterations < max_iterations; ++iterations) {
      if (x < 1.5 || y < 1.5 || (x + 1.5) > s.width || (y + 1.5) > s.height) {
        if (debug) printf("out of bounds %7.2f, %7.2f\n", x, y);
        pt->x = x;
        pt->y = y;
        return OUT_OF_BOUNDS;
      }
      // Get the interpolated image and gradients about this
      // point.
      Patch np = GetPatch(gimg, cv::Point2f(x, y));

      // Compute scale factors.
      float alpha = sqrt(patch.sumsq / np.sumsq);
      float beta = patch.mean - alpha * np.mean;

      alpha = 1.; beta = 0;
      // Compute Gxx, Gxy, Gyy and Ex, Ey.
      float gxx(0), gxy(0), gyy(0);
      float ex(0), ey(0);
      int count(0);
      for (int i = 0; i < len_; ++i) {
        if (np.data[i] == 0 || patch.data[i] == 0)
          continue;  // portion of patch was out of bounds.
        ++count;
        float diff = patch.data[i] - np.data[i] * alpha - beta;


        diff = ((diff < 0) ? -1 : 1) * sqrt(fabs(diff));

        float gradx = patch.gradx[i] + np.gradx[i] * alpha;
        float grady = patch.grady[i] + np.grady[i] * alpha;
        
        gxx += gradx * gradx;
        gxy += gradx * grady;
        gyy += grady * grady;

        ex += diff * gradx;
        ey += diff * grady;
      }

      //ex *= 1./255.;
      //ey *= 1./255.;

      // Compute determinant.
      float det = gxx*gyy - gxy*gxy;
	
      // Break if the determinant is too small.
      if (det < threshold) {
        if (debug) printf("Small det\n");
        return SMALL_DET;
      }

      // Compute shift
      float dx = (gyy * ex - gxy * ey)/det;
      float dy = (gxx * ey - gxy * ex)/det;

      float ddx = odx - dx;
      float ddy = ody - dy;
      float xscale = 1.;
      float yscale = 1.;

      if (xvalid && fabs(ddx) < 0.3 && fabs(ddx) > 0) {
        xscale = dx / ddx;
        xvalid = false;
      } else {
        xvalid = true;
      }

      if (yvalid && fabs(ddy) < 0.3 && fabs(ddy) > 0) {
        yscale = dy / ddy;
        yvalid = false;
      } else {
        yvalid = true;
      }

      if (debug)
      printf(" * [%7.2f, %7.2f] + [%7.2f, %7.2f] scale %7.2f, %7.2f count %d\n", x, y, dx, dy, xscale, yscale, count);
      //if (debug)
      //printf("   grad xx xy yy %7.2f %7.2f %7.2f ex ey %7.2f, %7.2f\n", gxx, gxy, gyy, ex, ey);
      xscale = max(1.f, min(200.f, xscale));
      yscale = max(1.f, min(200.f, yscale));
      xscale = min(xscale, yscale);
      // Update current point.
      x += max(-2.f, min(2.f, dx * xscale));
      y += max(-2.f, min(2.f, dy * yscale));
      odx = dx;
      ody = dy;
      if (fabs(dx) < threshold && fabs(dy) < threshold)
        break;
    }
      if (debug)
    printf(" * [%7.2f, %7.2f] => [%7.2f, %7.2f]\n", pt->x, pt->y, x, y);

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

    cv::Point2f p = (*pt) * (1./ (1 << (lvls - 1)));
    for (int i = lvls - 1 ; i > 0; --i) {
      Status status = Track(stack[i], patches[i], threshold * 10, max_iterations, &p);
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
};

#endif
