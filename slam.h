/*
 * slam.h
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

#ifndef SLAM_H_
#define SLAM_H_

#include "localmap.h"

Vector2d fposToVector(const FPos& fp);
FPos vectorToFPos(const Vector2d& v);

void RunSlam(LocalMap* map, int min_frame_to_solve);

class Slam {
};

#endif /* SLAM_H_ */
