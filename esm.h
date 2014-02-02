/*
 * esm.h
 *
 *  Created on: 10/02/2013
 *      Author: moreil
 */

#ifndef ESM_H_
#define ESM_H_
// Compute homography between reference image and target image using
// efficient second-order matching.

#include <eigen3/Eigen/Eigen>

class Esm {
 public:
  Esm();
  virtual ~Esm();

  void run_esm(const SubImage& ref_image, const Subimage& target, Homography* h) {
    // Compute image gradients for ref image.
    Matrix<uint16_t, Dynamic, Dynamic> x_grad(ref_image.width(), ref_image.height());
    Matrix<uint16_t, Dynamic, Dynamic> y_grad(ref_image.width(), ref_image.height());
    for (auto& p : ref_image.region()) {
      x_grad(p.x, p.y) = (uint16_t)ref_image.pixel(p + Pos(1, 0)) - ref_image.pixel(p - Pos(1, 0));
      y_grad(p.x, p.y) = (uint16_t)ref_image.pixel(p + Pos(0, 1)) - ref_image.pixel(p - Pos(0, 1));
    }

  }
};

#endif /* ESM_H_ */
