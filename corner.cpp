/*
 * corner.cpp
 *
 *  Created on: 04/02/2012
 *      Author: moreil
 */

#include "octave.h"
#include "util.h"

#include "corner.h"

// Low numbered ids reserved for keyframes.
int Corner::next_id_ = 1000;

const uint8_t guass_weights[] = {
    4,   6,   9,  11,  11,   9,   6,   4,
    6,  11,  15,  18,  18,  15,  11,   6,
    9,  15,  21,  25,  25,  21,  15,   9,
   11,  18,  25,  30,  30,  25,  18,  11,
   11,  18,  25,  30,  30,  25,  18,  11,
    9,  15,  21,  25,  25,  21,  15,   9,
    6,  11,  15,  18,  18,  15,  11,   6,
    4,   6,   9,  11,  11,   9,   6,   4,
};
