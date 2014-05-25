#ifndef PROJECT_H_
#define PROJECT_H_

#include <eigen3/Eigen/Eigen>

template<typename T>
double jtod(const T& v) {
  return *((double*)(&v));
}

struct ProjectPoint {
  template <typename T>
  bool operator()(
      const T* const frame_rotation,  // [x,y,z,w] (eigen quaternion)
      const T* const frame_translation,  // [x,y,z]
      const T* const intrinsics,  // [k1, k2, k3, fx, fy, cx, cy]
      const T* const point,  // homogenous coordinates. [x,y,z,w]
      T* result) const {
    Eigen::Map<const Eigen::Quaternion<T> > q(frame_rotation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > translate(frame_translation);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > mpoint(point);

    // Compute rotated translated point in [x*w, y*w, z*w] space.
    Eigen::Matrix<T, 3, 1> p = q * (mpoint - translate * point[3]);

    // Don't project points that are effectively behind the intrinsics lens.
    if (p[2] < 0.001 * point[3]) {
      printf("Fail point [%f, %f, %f]\n", jtod(p[0]), jtod(p[1]), jtod(p[2]));
      return false;
    }

    // Project onto the image plane.
    // There isn't a '/ point[3]' as it cancels out: The full expression is
    // (p[0]/point[3]) / (p[2]/point[3])
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    T r2 = xp*xp + yp*yp;
    T distort = T(1.0) + r2 * (intrinsics[0] + r2 * (intrinsics[1] + r2 * intrinsics[2]));
    xp *= distort;
    yp *= distort;

    xp *= intrinsics[3];
    yp *= intrinsics[4];

    xp += intrinsics[5];
    yp += intrinsics[6];

    // Compute final projected point position.
    result[0] = xp;
    result[1] = yp;
    return true;
  }
};


#endif
