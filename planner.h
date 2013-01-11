/*
 * planner.h
 *
 *  Created on: Jan 1, 2013
 *      Author: michael
 */

/*
 * Actuators: steering front/rear, throttle commanded settings.
 * State: position, velocity, accel, wheel angles.
 * WorldMap: legal positions in the world.
 * StateMap: legal positions in the state space.
 */
#ifndef PLANNER_H_
#define PLANNER_H_

#include <eigen3/Eigen/Eigen>

#include "grid.h"

using namespace Eigen;

class Planner {
public:
  static Vector2d rotate_left(const Vector2d& p) {
    Vector2d r;
    r << -p(1), p(0);
    return r;
  }

  static Vector2d rotate_right(const Vector2d& p) {
    Vector2d r;
    r << p(1), -p(0);
    return r;
  }

};

void planWithSimpleSetup(void);


#endif /* PLANNER_H_ */
