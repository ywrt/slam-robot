#ifndef KLT_H_
#define KLT_H_

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

extern int debug;

class KLTTracker {
  public:
    KLTTracker(cv::Size size) : size_(size), len_(size.width * size.height) {
      mask_.resize(len_);
      for (int y = 0; y < size.width; ++y)  {
        for (int x = 0; x < size.width; ++x)  {
          double rx = (0.5 * size.width - x);
          double ry = (0.5 * size.height- y);
          double rr = rx * rx + ry * ry;
          mask_[y * size.width + x] = 1. / (15. + rr);
          printf("%7.4f ", 1. / (15. + rr));
        }
        printf("\n");
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
    Sobel(g.image, g.gradx, CV_32F, 1, 0, CV_SCHARR, 1./32.);
    Sobel(g.image, g.grady, CV_32F, 0, 1, CV_SCHARR, 1./32.);

    result[0] = g;
    for (int i = 1 ; i < depth; ++i) {
      GradImage& curr = result[i];
      GradImage& prev = result[i - 1];
      cv::Size s((prev.image.cols + 1) / 2, (prev.image.rows + 1) / 2);

      curr.image = cv::Mat(s, CV_32F);
      curr.gradx = cv::Mat(s, CV_32F);
      curr.grady = cv::Mat(s, CV_32F);

      cv::pyrDown(prev.image, curr.image);
      cv::GaussianBlur(curr.image, curr.image, cv::Size(5, 5), .6, .6);
      Sobel(curr.image, curr.gradx, CV_32F, 1, 0, 1, 1./32.);
      Sobel(curr.image, curr.grady, CV_32F, 0, 1, 1, 1./32.);
#if 1
      cv::pyrDown(prev.gradx, curr.gradx); curr.gradx *= 2.f;
      cv::pyrDown(prev.grady, curr.grady); curr.grady *= 2.f;
#endif
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

  float SADPatches(const Patch& p1, const Patch& p2) {
    float sum = 0;
    for (int i = 0; i < len_; ++i) {
      if (p1.data[i] == 0 || p2.data[i] == 0)
        continue;
      float diff = p1.data[i] - p2.data[i];
      //sum += fabs(diff);
      sum += diff*diff * mask_[i];
    }
    return sum;
  }

  float SearchBest(const GradImage& g, const Patch& patch, cv::Point2f pt, float window, float* dx, float* dy) {
    float best_sad = 1e6;

    for (float x = -window ; x <= window; x += 0.01) {
      for (float y = -window ; y <= window; y += 0.01) {
        Patch np = GetPatch(g, cv::Point2f(pt.x + x, pt.y + y));
        float sad = SADPatches(np, patch);
        if (sad > best_sad)
          continue;
        *dx = x;
        *dy = y;
        best_sad = sad;
      }
    }
    return best_sad;
  }

  void BruteGradient(const GradImage& g, const Patch& patch, cv::Point2f pt, float* dx, float* dy) {
    const float h = 0.001;
    float sad0 = SADPatches(patch, GetPatch(g, pt));
    float sadx = SADPatches(patch, GetPatch(g, cv::Point2f(pt.x + h, pt.y)));
    float sady = SADPatches(patch, GetPatch(g, cv::Point2f(pt.x, pt.y + h)));

    float dx0 = (sadx - sad0) / h;
    float dy0 = (sady - sad0) / h;

    *dx = dx0;
    *dy = dy0;
  }

  void BruteHessian(const GradImage& g, const Patch& patch, cv::Point2f pt,
      float * dx,
      float * dy,
      float* dxx,
      float* dxy,
      float* dyx,
      float* dyy) {
    const double h = 0.01;
    double sad0 = SADPatches(patch, GetPatch(g, pt));
    double sadx = SADPatches(patch, GetPatch(g, cv::Point2f(pt.x + h, pt.y)));
    double sady = SADPatches(patch, GetPatch(g, cv::Point2f(pt.x, pt.y + h)));

    double sadxx = SADPatches(patch, GetPatch(g, cv::Point2f(pt.x + 2 * h, pt.y)));
    double sadyy = SADPatches(patch, GetPatch(g, cv::Point2f(pt.x, pt.y + 2 * h)));
    double sadxy = SADPatches(patch, GetPatch(g, cv::Point2f(pt.x + h, pt.y + h)));

    *dx = (sadx - sad0) / h;
    *dy = (sady - sad0) / h;

    *dxx = ((sadxx - sadx) / h - (sadx - sad0) / h) / h;
    *dyy = ((sadyy - sady) / h - (sady - sad0) / h) / h;
    *dxy = ((sadxy - sady) / h - (sadx - sad0) / h) / h;
    *dyx = ((sadxy - sadx) / h - (sady - sad0) / h) / h;
  }

  void BruteQuadratic(const GradImage& g, const Patch& patch, cv::Point2f pt, float* dx, float* dy, float* s) {
    const float h = 0.01;
    Patch p0 = GetPatch(g, pt);
    float sad0 = SADPatches(patch, p0);

    Patch px = GetPatch(g, cv::Point2f(pt.x + h, pt.y));
    float sadx = SADPatches(patch, px);

    Patch py = GetPatch(g, cv::Point2f(pt.x, pt.y + h));
    float sady = SADPatches(patch, py);

    float dx0 = (sadx - sad0) / h;
    float dy0 = (sady - sad0) / h;
    float norm = sqrt(dx0 * dx0 + dy0 * dy0) + 1e-9;

    float deltax = dx0 / norm * h;
    float deltay = dy0 / norm * h;

    float sad1 = SADPatches(patch, GetPatch(g, pt + cv::Point2f(-deltax, -deltay)));
    float sad2 = SADPatches(patch, GetPatch(g, pt + cv::Point2f(deltax, deltay)));

    // Solve the quadratic.
    float ds1 = (sad0 - sad1) / h;
    float ds2 = (sad2 - sad0) / h;
    float dds = (ds2 - ds1) / h;

    if (debug) printf(" ds [%9.5f, %9.5f], dds %9.5f (dx,dy %f, %f : deltax %f deltay %f)\n", ds1, ds2, dds, dx0, dy0, deltax, deltay);
    float scale = 0.1f;
    if (dds > 1e-6) {
      scale = (ds1 + ds2) * 0.5f / (2.f * dds + 1.f);
      dx0 = deltax * scale / h;
      dy0 = deltay * scale / h;
    }  else {
      dx0 *= 0.5f;
      dy0 *= 0.5f;
    }

    *s = scale;
    *dx = -dx0 ;
    *dy = -dy0 ;
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

    //float wdx(-1), wdy(-1), wbest(-1);
    //if (debug) wbest = SearchBest(gimg, patch, cv::Point2f(x, y), 4, &wdx, &wdy);

    const float margin = 0.1;
    cv::Size s = gimg.image.size();
    float lambda = .0001;
    float last_sad = 1e3;
    for (int iterations = 0; iterations < max_iterations; ++iterations) {
      if (x < margin || y < margin || (x + margin) > s.width || (y + margin) > s.height) {
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

      //alpha = 1.; beta = 0;

      Eigen::Matrix2f A, B, C;
      A = B = C = Eigen::Matrix2f::Zero();

      Eigen::Vector2f RS, VW;
      RS = VW = Eigen::Vector2f::Zero();

      int count(0);
      float sad(0);
      for (int i = 0; i < len_; ++i) {
        if (np.data[i] == 0 || patch.data[i] == 0)
          continue;  // portion of patch was out of bounds.
        ++count;

        float I = patch.data[i];
        float J = np.data[i] * alpha + beta;
        sad += fabs(I - J)*mask_[i];

        Eigen::Vector2f gI, gJ;
        gI << patch.gradx[i], patch.grady[i];
        gJ << np.gradx[i] * alpha, np.grady[i] * alpha;

        // Equation 15 from the paper.
        A += gI * gI.transpose() * mask_[i];
        B += gI * gJ.transpose() * mask_[i];
        C += gJ * gJ.transpose() * mask_[i];
        float diff = (I - J) * mask_[i];
        RS += diff * gI;
        VW += diff * gJ;
      }

      // In the paper they show a D matrix, but it is just B transpose, so use that
      // instead of explicitly computing D.
      Eigen::Matrix2f Di = B.transpose().inverse();

      // Equation 14 from the paper.
      //Eigen::Matrix2f U = A*Di*C + lambda*Di*C - 0.5*B;
      Eigen::Matrix2f U = (A + lambda * Eigen::Matrix2f::Identity()) * Di * C - 0.5 * B;
      Eigen::Vector2f e = (A + lambda * Eigen::Matrix2f::Identity()) * Di * VW - 0.5 * RS;
#if 0
      // Forward + backward KLT
      U = (A + B + C + B.transpose()) * 0.5;
      e = (RS + VW) * 0.5;

      // Pure forward KLT
      U = A;
      e = RS;
#endif

      // Ud = e. Solve for d.
      Eigen::Vector2f d = U.lu().solve(e);

      // Break if the determinant is too small.
//      if (det < threshold) {
 //       if (debug) printf("Small det\n");
 //       return SMALL_DET;
 //     }

      // Compute shift
      float dx = d[0] * 0.5;
      float dy = d[1] * 0.5;

#if 1
      float mdx(-1), mdy(-1), mdxx(-1), mdxy(-1), mdyx(-1), mdyy(-1);
      BruteHessian(gimg, patch, cv::Point2f(x, y), &mdx, &mdy, &mdxx, &mdxy, &mdyx, &mdyy);
      //BruteGradient(gimg, patch, cv::Point2f(x, y), &mdx, &mdy);

      Eigen::Matrix2d H;
      H << mdxx, mdxy, mdyx, mdyy;
      Eigen::Vector2d g;
      g << mdx, mdy;
      //std::cout << H << "\n";


      Eigen::Vector2d jj = H.inverse() * g;

      if (debug)
      printf(" * [%7.2f, %7.2f] + [%7.2f, %7.2f] beta [%7.2f, %7.2f] J [%7.2f, %7.2f] sad %6.4f rs [%6.2f, %6.2f]\n",
          x, y, dx, dy, -jj(0), -jj(1), g[0], g[1], sad, -RS[0], -RS[1]);

      dx = -jj(0);
      dy = -jj(1);

      if ((dx*dx + dy*dy) > 1) {
        dx /= sqrt(dx * dx + dy * dy);
        dy /= sqrt(dx * dx + dy * dy);
      }
#endif
    //  if (debug) printf(" * [%7.2f, %7.2f] + [%7.2f, %7.2f] sad %6.4f rs [%6.2f, %6.2f]\n",
     //     x, y, dx, dy, sad, RS[0], RS[1]);
      if (sad > last_sad) {
  //      if (debug) printf("Sad increase.\n");
 //       break;
      }
      last_sad = sad;

      // Update current point.
      x += max(-1.f, min(1.f, dx));
      y += max(-1.f, min(1.f, dy));
      if (fabs(dx) < threshold/10. && fabs(dy) < threshold/10.)
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

    cv::Point2f p = (*pt) * (1./ (1 << (lvls - 1)));
    for (int i = lvls - 1 ; i > 0; --i) {
      Status status = Track(stack[i], patches[i], threshold * 50, max_iterations, &p);
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
