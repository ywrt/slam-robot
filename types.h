#ifndef TYPES_H_
#define TYPES_H_
// Various common types.

#include <eigen3/Eigen/Eigen>

// TODO: Define types for various spaces.
// [0,1]x[0,1]
// [-1, 1]x[-1, 1]
// [0,rows]x[0,cols]

class PixelPt : public Eigen::Vector2d {
 public:
  PixelPt(double x, double y) { size_ << x << y; }
  PixelPt(const Vector2d& size) : size_(size) { }

  double x() const { return this->[0] / size_[0]; }
  double y() const { return this->[1] / size_[1]; }

 private:
  Vector2d size_;
};

class FramePt : public Eigen::Vector2d {
  
}


#endif
