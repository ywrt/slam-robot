#ifndef PROJECT_H_
#define PROJECT_H_

#include <eigen3/Eigen/Eigen>

struct ProjectPoint {
  template <typename T>
  bool operator()(
      const T* const xyscale,  // [xscale, yscale]
      const T* const distortion,  // [r^2, r^4]

      const T* const frame_rotation,  // [x,y,z,w] (eigen quaternion)
      const T* const frame_translation,  // [x,y,z]

      const T* const point,  // homogenous coordinates. [x,y,z,w]
      T* result) const {
    Eigen::Map<const Eigen::Quaternion<T> > q(frame_rotation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > translate(frame_translation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > mpoint(point);

    // Compute rotated translated point in [x*w, y*w, z*w] space.
    Eigen::Matrix<T, 3, 1> p = q * mpoint + translate * point[3];

    // Don't project points that are effectively behind the camera lens.
    if (p[2] < 0.001)
      return false;

    // Project onto the image plane.
    // There isn't a '/ point[3]' as it cancels out: The full expression is
    // (p[0]/point[3]) / (p[2]/point[3])
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
  
    // Apply second and fourth order radial distortion.
    const T& l1 = distortion[0];
    const T& l2 = distortion[1];
    T r2 = xp*xp + yp*yp;  // radius^2
    T scale = (T(1.0) + r2 * (l1 + l2 * r2));

    // now project onto pixel plane.
    xp *= scale * xyscale[0];
    yp *= scale * xyscale[1];

    // Compute final projected point position.
    result[0] = xp;
    result[1] = yp;
    return true;
  }
};


#endif
