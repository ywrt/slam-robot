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

void planWithSimpleSetup(void);


#endif /* PLANNER_H_ */
