#ifndef BRUTE_H_
#define BRUTE_H_

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

extern int debug;

class BruteTracker {
  public:
    BruteTracker(cv::Size size) : size_(size), len_(size.width * size.height) {}

  struct Patch {
    Patch(cv::Size s) :
       size(s), len(s.width * s.height), data(len) {}

    cv::Size size;
    int len;
    vector<float> data;
    float mean;
    float sumsq;
  };

  struct GradImage {
    cv::Mat image;
  };

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
      patch.data[i] = 0;
    }

    cv::getRectSubPix(
        gimg.image,
        size_,
        pt,
        cv::Mat(size_.width, size_.height, CV_32F, &patch.data[0], size_.width * sizeof(float))); 

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

    result[0] = g;
    for (int i = 1 ; i < depth; ++i) {
      GradImage& curr = result[i];
      GradImage& prev = result[i - 1];
      cv::Size s((prev.image.cols + 1) / 2, (prev.image.rows + 1) / 2);

      curr.image = cv::Mat(s, CV_32F);

      cv::pyrDown(prev.image, curr.image);
    }

    CHECK_EQ(CV_32F, result[0].image.type());
    return result;
  }

  float SADPatches(const Patch& p1, const Patch& p2) {
    float alpha = sqrt(p1.sumsq / p2.sumsq);
    float beta = p1.mean - alpha * p2.mean;

    float sum = 0;
    for (int i = 0; i < len_; ++i) {
      if (p1.data[i] == 0 || p2.data[i] == 0)
        continue;
      float diff = p1.data[i] - p2.data[i] * alpha - beta;
      sum += fabs(diff * diff);
    }
    return sum;
  }

  float SearchBest(
      const GradImage& g,
      const Patch& patch,
      float window,
      float res,
      cv::Point2f* pt) {
    float best_sad = 1e6;
    cv::Point2f p = *pt;

    for (float x = -window ; x <= window; x += res) {
      for (float y = -window ; y <= window; y += res) {
        float sad = SADPatches(patch, GetPatch(g, cv::Point2f(p.x + x, p.y + y)));
        if (sad > best_sad)
          continue;
        pt->x = p.x + x;
        pt->y = p.y + y;
        best_sad = sad;
      }
    }
    printf("  * [%7.2f, %7.2f] + [%7.2f, %7.2f] => %7.4f\n", p.x, p.y, pt->x - p.x, pt->y - p.y, best_sad);
    return best_sad;
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

  Status TrackFeature(
      const vector<GradImage>& stack,
      const vector<Patch>& patches,
      float threshold,
      int max_iterations,
      cv::Point2f* pt) {
    int lvls = stack.size();

    const float margin = 13;
    cv::Size s = stack[0].image.size();
    if (pt->x < margin || pt->y < margin || (pt->x + margin) > s.width || (pt->y + margin) > s.height) {
      if (debug) printf("out of bounds %7.2f, %7.2f\n", pt->x, pt->y);
      return OUT_OF_BOUNDS;
    }

    cv::Point2f p = (*pt) * (1./ (1 << (lvls - 1)));
    for (int i = lvls - 1 ; i > 0; --i) {
      float sad(0);
      sad = SearchBest(stack[i], patches[i], 3, 1, &p);
      sad = SearchBest(stack[i], patches[i], 1, 0.33333, &p);
      if (sad > 100)
        return OUT_OF_BOUNDS;
      p *= 2.;  // Upscale for the next larger image.
    }
    float sad(0);
    sad = SearchBest(stack[0], patches[0], 3, 1, &p);
    sad = SearchBest(stack[0], patches[0], 1, 0.3333, &p);
    sad = SearchBest(stack[0], patches[0], 0.4, 0.1, &p);
    sad = SearchBest(stack[0], patches[0], 0.2, 0.025, &p);
    sad = SearchBest(stack[0], patches[0], 8, 0.01, &p);
    if (sad > 100)
      return OUT_OF_BOUNDS;

    *pt = p;
    return OK;
  }

 private:
  cv::Size size_;
  int len_;
};

#endif
