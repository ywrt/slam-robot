/*
 * planner.cpp
 *
 *  Created on: Jan 1, 2013
 *      Author: michael
 */

#include "planner.h"

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

void planWithSimpleSetup(void) {
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

  auto planner = new oc::KPIECE1(ss.getSpaceInformation());
  planner->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();
  ss.setPlanner(ompl::base::PlannerPtr(planner));

  bool solved = ss.solve(10.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
   // print the path to screen
 //  ss.simplifySolution();
   ss.getSolutionPath().print(std::cout);
  }
}
