/*
 * planner.cpp
 *
 *  Created on: Jan 1, 2013
 *      Author: michael
 */

#include "planner.h"

#include <string>
#include <iostream>

#include <eigen3/Eigen/Eigen>


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace Eigen;

const double kTurningRadius = 2;

struct State {
  Vector2d pos_;
  double direction_;
};

struct Segment {
  Segment(double dist, int type) :
    distance_(dist), type_(type) {}

  double distance_;  // positive or negative.
  int type_;  // -1 left, 0 straight, 1 right;
};

double mod2pi(double a) {
  while (a < 0) a += 2 * M_PI;
  while (a > M_PI * 2) a -= 2 * M_PI;
  return a;
}

vector<Segment> generate_LSL(const State& curr,
                             const State& goal,
                             int parity) {
  vector<Segment> segments;

  Vector2d offset = Rotation2D<double>(curr.direction_
                                       + parity * M_PI_2) *
      Vector2d::UnitX();
  Vector2d ca = curr.pos_ + kTurningRadius * offset;

  offset = Rotation2D<double>(goal.direction_
                              + parity * M_PI_2) *
      Vector2d::UnitX();
  Vector2d cb = goal.pos_ + kTurningRadius * offset;

  Vector2d heading = cb - ca;
  double dist = heading.norm();
  heading.normalize();

  if (dist < kTurningRadius * 2)
    return segments;  // Can't compute path as circles intersect.

  double angle = atan2(heading(1), heading(0));

  // first circle angle.
  double a1 = angle - curr.direction_;
  double a2 = goal.direction_ - angle;
  
  segments.push_back(Segment(mod2pi(parity * a1), -1 * parity));
  segments.push_back(Segment(dist, 0));
  segments.push_back(Segment(mod2pi(parity * a2), -1 * parity));
  return segments;
}


vector<Segment> generate_LSR(const State& curr,
                             const State& goal,
                             int parity) {
  vector<Segment> segments;

  Vector2d ca = curr.pos_ + kTurningRadius
      * (Rotation2D<double>(curr.direction_ + parity * M_PI_2)
      * Vector2d::UnitX());

  Vector2d cb = goal.pos_ + kTurningRadius
        * (Rotation2D<double>(goal.direction_ - parity * M_PI_2)
        * Vector2d::UnitX());

  Vector2d heading = cb - ca;
  double dist = heading.norm();

  if (dist < kTurningRadius * 2)
    return segments;  // Can't compute path as circles intersect.

  heading.normalize();
  double angle = atan2(heading(1), heading(0));

  // There is a triangle formed by the mid-point of the line
  // between the two centers, one of the centers, and the
  // tangent. The length of the line between the two centers
  // is known, as is the radius, so we can compute the angle
  // between the line joining the centers, and the tangent to
  // the circles.
  double theta = asin(kTurningRadius / (dist / 2));

  // Similarly, we can compute the length of the tangent
  // between the two circles.
  double tdist = sqrt(dist*dist -
                          4 * kTurningRadius*kTurningRadius);

  // The heading when the car leaves the circle is then:
  double angle1 = angle + parity * theta;

  // So the car travels this distance on the first circle
  // (in radians).
  double a1 = angle1 - curr.direction_;

  // And then this distance on the second circle.
  double a2 = angle1 - goal.direction_;

  segments.push_back(Segment(mod2pi(a1 * parity), -1 * parity));
  segments.push_back(Segment(tdist, 0));
  segments.push_back(Segment(mod2pi(a2 * parity), 1 * parity));
  return segments;
}


// Two circles, possibly intersecting.
// We trace the path along a 3rd circle that joins them.
vector<Segment> generate_LRL(const State& curr,
                             const State& goal,
                             int parity) {
  vector<Segment> segments;

  Vector2d ca = curr.pos_ + kTurningRadius
      * (Rotation2D<double>(curr.direction_ + parity * M_PI_2)
      * Vector2d::UnitX());

  Vector2d cb = goal.pos_ + kTurningRadius
        * (Rotation2D<double>(goal.direction_ + parity * M_PI_2)
        * Vector2d::UnitX());

  Vector2d heading = cb - ca;
  double dist = heading.norm();

  if (dist > kTurningRadius * 4)
    return segments; // Circles don't intersect.

  double angle = atan2(heading(1), heading(0));

  // Triangle formed by the centers of the 3 circles.
  // Distance between first two centers is 'dist'.
  // Distance between from those to the other center is
  // 2 * kTurningRadius.
  double theta = acos((dist/2) / (kTurningRadius*2));

  printf("theta %f\n", theta / M_PI * 180.);

  double t1 = curr.direction_ - angle - M_PI_2;
  double t2 = goal.direction_ - angle - M_PI_2;

  double a1 = theta - t1;
  double a2 = M_PI + 2 * theta;
  double a3 = t2 - (M_PI - theta);

  segments.push_back(Segment(mod2pi(a1), -1));
  segments.push_back(Segment(mod2pi(a2), 1));
  segments.push_back(Segment(mod2pi(a3), -1));

  return segments;
}

vector<Segment> reverse_path(const vector<Segment>& segments) {
  vector<Segment> out;
  for (int i = segments.size() - 1 ; i >= 0 ; --i) {
    const Segment& s = segments[i];
    if (s.type_ == 0) {
      out.push_back(Segment(s.distance_, s.type_));
    } else {
      out.push_back(Segment(mod2pi(-segments[i].distance_),
                            segments[i].type_ * -1));
    }
  }
  return out;
}

vector<Segment> generate_path(const State& curr,
                              const State& goal,
                              int type) {
  switch(type) {
    case 0:
      return generate_LSL(curr, goal, 1);
    case 1:
      return generate_LSR(curr, goal, 1);
    case 2:
      return generate_LSL(curr, goal, -1);
    case 3:
      return generate_LSR(curr, goal, -1);
    case 4:
      return generate_LRL(curr, goal, 1);
  }
}

vector<Vector2d> interpolate_path(const State& curr,
                           const vector<Segment>& segments,
                           double step) {
  State c = curr;
  vector<Vector2d> path;

  for (auto& s : segments) {
    printf("%3d : %f\n", s.type_, s.distance_);
    path.push_back(c.pos_);  // Push start.
    if (s.type_ == 0) {
      // Straight line.
      Vector2d heading =
          Rotation2D<double>(c.direction_) * Vector2d::UnitX();
      for (double t = step; t < s.distance_; t += step) {
        path.push_back(t * heading + c.pos_);
      }
      c.pos_ += s.distance_ * heading;
    } else {
      // Left or right curve.
      Vector2d center = c.pos_ + kTurningRadius
          * (Rotation2D<double>(c.direction_ - s.type_ * M_PI_2)
          * Vector2d::UnitX());
      double t1 = c.direction_ - s.type_ * M_PI_2 + M_PI;
      for (double t = step ; t < s.distance_; t += step) {
        double angle = t1 - t * s.type_;
        Vector2d p = center + kTurningRadius *
            (Rotation2D<double>(angle) * Vector2d::UnitX());
        path.push_back(p);
      }

      c.pos_ = center + kTurningRadius *
          (Rotation2D<double>(t1 - s.distance_ * s.type_)
                    * Vector2d::UnitX());

      c.direction_ -= s.type_ * s.distance_;
    }
  }
  path.push_back(c.pos_);

  return path;
}

// project [-10,10] -> [0, mat.rows] etc.
cv::Point rescale(const cv::Mat& mat, const Vector2d& p) {
  double x = (p(0) + 10) / 20 * mat.cols;
  double y = (10 - p(1)) / 20 * mat.rows;
  return cv::Point(x, y);
}

void draw(cv::Mat& mat, const vector<Vector2d>& path) {
  printf("[%f, %f] -> [%f, %f]\n",
         path[0](0),path[0](1),
         path.back()(0), path.back()(1));
  for (size_t i = 0; i < path.size() - 1 ; ++i) {
    const Vector2d& p1 = path[i+0];
    const Vector2d& p2 = path[i+1];

    cv::Point a1(rescale(mat, p1));
    cv::Point a2(rescale(mat, p2));

    line(mat, a1, a2, cv::Scalar(255 - i, i, i), 1, 8);
  }

  cv::imshow("path", mat);
 // cv::waitKey(1000);
}

void planWithSimpleSetup(void) {
  State goal;
  goal.pos_ << 0,-3;
  goal.direction_ = 0;

  State curr;
  curr.pos_ << 0,0;
  curr.direction_ = 0;
  cv::Mat mat(1024, 1024, CV_8UC3);

  mat = cv::Scalar(0,0,0);

  for (double t = .5 ; t < 4.5; t += 0.1) {
    goal.pos_(1) = -t;
    vector<Segment> segments = generate_path(curr, goal, 4);
    vector<Vector2d> path = interpolate_path(curr, segments, 0.1);
    draw(mat, path);
  }
  cv::waitKey(0);

}

