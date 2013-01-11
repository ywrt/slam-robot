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


#include <ompl/base/State.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;

bool isStateValid(const ob::State *state) {
  return true;
}

void postPropagate(const oc::Control* control,
                   const ob::State* state) {
#if 0
  ob::SO2StateSpace SO2;
  // Ensure that the car's resulting orientation lies between 0 and 2*pi.
  ob::SE2StateSpace::StateType& s =
      *result->as<ob::SE2StateSpace::StateType>();
  SO2.enforceBounds(s[1]);
#endif
}


void SimpleCarODE(const oc::ODESolver::StateType& q,
                  const oc::Control* c,
                  oc::ODESolver::StateType& qdot) {
  // Retrieve control values. Velocity is the first entry, steering angle is second.
  const double *u =
      c->as<oc::RealVectorControlSpace::ControlType>()->values;
  const double velocity = u[0];
  const double steeringAngle = u[1];
  // Retrieve the current orientation of the car. The memory for ompl::base::SE2StateSpace is mapped as:
  // 0: x
  // 1: y
  // 2: theta
  const double theta = q[2];
  // Ensure qdot is the same size as q. Zero out all values.
  qdot.resize(q.size(), 0);
  qdot[0] = velocity * cos(theta); // x-dot
  qdot[1] = velocity * sin(theta); // y-dot
  qdot[2] = velocity * tan(steeringAngle); // theta-dot
}

void planOMPLWithSimpleSetup(void) {
  // construct the state space we are planning in
  ob::StateSpacePtr spacex(new ob::DubinsStateSpace());
  ob::RealVectorBounds bounds(2);
  bounds.setLow(-1);
  bounds.setHigh(1);
  spacex->as<ob::SE2StateSpace>()->setBounds(bounds);

  oc::ControlSpacePtr control(
      new oc::RealVectorControlSpace(spacex, 2));
  control->as<oc::RealVectorControlSpace>()->setBounds(bounds);

  oc::SimpleSetup ss(control);
  ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

  // SpaceInformationPtr is defined as the variable si.
  oc::ODESolver* odeSolver(
      new ompl::control::ODEBasicSolver<>(
          ss.getSpaceInformation(), &SimpleCarODE));

  ss.setStatePropagator(
      odeSolver->getStatePropagator(&postPropagate));

  ob::ScopedState<ob::SE2StateSpace> start(spacex);
  start->setX(-0.5);
  start->setY(0.0);
  start->setYaw(0.0);

  ob::ScopedState<ob::SE2StateSpace> goal(spacex);
  goal->setX(0.0);
  goal->setY(0.5);
  goal->setYaw(0.0);

  ss.setStartAndGoalStates(start, goal, 0.05);

  //auto planner = new oc::KPIECE1(ss.getSpaceInformation());
  //planner->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();
  //ss.setPlanner(ompl::base::PlannerPtr(planner));

  bool solved = ss.solve(10.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
   // print the path to screen
 //  ss.simplifySolution();
   ss.getSolutionPath().print(std::cout);
  }
}

std::string string_format(const std::string &fmt, ...) {
    int size = 100;
    std::string str;
    va_list ap;
    while (1) {
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
        va_end(ap);
        if (n > -1 && n < size) {
            str.resize(n);
            return str;
        }
        if (n > -1)
            size = n + 1;
        else
            size *= 2;
    }
    return str;
}

struct State {
  Vector2d pos_;
  double direction_;
  double velocity_;
  double steering_;
  string ToString() const {
    return string_format(
        "[pos %5.3f %5.3f dir %5.3f vel %5.3f steer %5.3f]",
        pos_(0), pos_(1),
        direction_,
        velocity_,
        steering_);
  }
};

double clamp(double v, double range) {
  if (v < -range) v = -range;
  if (v > range) v = range;
  return v;
}

struct Control {
  Control() : steering_(0), accel_(0) {}
  Control(double steering, double accel) :
    steering_(steering), accel_(accel) {}

  void add(const Control& delta, double scale) {
    steering_ = clamp(steering_ + scale * delta.steering_, M_PI*.9);
    accel_ = clamp(accel_ + scale* delta.accel_, 1);
  }
  double norm() const {
    return sqrt(steering_ * steering_ + accel_*accel_);
  }

  void normalize() {
    double n = norm();
    if (n == 0)
      return;
    steering_ /= n;
    accel_ /= n;
  }
  double steering_;
  double accel_;



};




State project(const State& current,
              const Control& control,
              double step) {
  State result = current;
  result.velocity_ += control.accel_ * step;
  result.steering_ += control.steering_ * step;

  result.steering_ = clamp(result.steering_, 1);

  result.direction_ += tan(result.steering_) * result.velocity_;

  while (result.direction_ < -M_PI) result.direction_ += M_PI * 2;
  while (result.direction_ > +M_PI) result.direction_ -= M_PI * 2;

  result.pos_(0) += cos(result.direction_) * result.velocity_;
  result.pos_(1) += sin(result.direction_) * result.velocity_;

  return result;
}


double distance(const State& s1, const State& s2) {
  double sum(0);
  sum += (s1.pos_ - s2.pos_).squaredNorm();

#if 1
  sum += (s1.velocity_ - s2.velocity_) * (s1.velocity_ - s2.velocity_);

  double ddir = s1.direction_ - s2.direction_;
  while (ddir < -M_PI) ddir += M_PI * 2;
  while (ddir > +M_PI) ddir -= M_PI * 2;
  sum += ddir * ddir;
#endif

  //cout << s1.ToString() << " v " << s2.ToString() << " = " << sum << endl;

  return sum;
}

double min_distance(const State& current,
                    const State& goal,
                    const Control& control,
                    double step) {
  State s = current;
  double dist = distance(current, goal);
  for (int i = 0; i < 100; ++i) {
    s = project(s, control, step);
    double d = distance(s, goal) + (0.05 * i);
    if (d < dist)
      dist = d;
  }
  return dist;
}

double deriv(const State& curr,
             const State& goal,
             const Control& control,
             const Control& delta) {

  Control c = control;
  c.add(delta, 1);
  double dist = min_distance(curr, goal, control, 1e-2);
  double min_dist = min_distance(curr, goal, c, 1e-2);

  return dist - min_dist;
}

Control gradient(const State& curr,
                 const State& goal,
                 struct Control& control
                 ) {

  double h = 1e-3;
  Control r(0,0);
  double best = 0;
  for (int i = -1 ; i < 2; ++i) {
    for (int j = -1 ; j < 2; ++j) {
      double d = deriv(curr, goal, control, Control(i*h, j*h));
      if (d < best)
        continue;
      best = d;
      if (d > 0) {
        r.steering_ = i;
        r.accel_ = j;
      }
    }
  }
  printf("%f, %f -> %f\n", r.steering_, r.accel_, best);

  r.normalize();
  return r;
}


void planWithSimpleSetup(void) {
  State goal;
  goal.pos_ << 5,0;
  goal.velocity_ = 0;
  goal.direction_ = 0;

  State curr;
  curr.pos_ << 0,0;
  curr.velocity_ = 0;
  curr.direction_ = 0;

  Control c(0,0);
  for (int i = 0; i < 25; ++i) {
    Control delta = gradient(curr, goal, c);

    c.add(delta, 0.1);

    curr = project(curr, c, 0.1);
    printf("%6.3f %6.3f @ vel %4.4f dir %5.2f (dir %6.3f accel %6.3f) [grad dir %f, accel %f]\n",
           curr.pos_(0), curr.pos_(1),
           curr.velocity_,
           curr.direction_,
           c.steering_,
           c.accel_,
           delta.steering_,
           delta.accel_
           );
  }



}

