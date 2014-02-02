#ifndef CORNERS_H_
#define CORNERS_H_

#include <vector>

class Octave;
namespace faster {
  class Corner;
}

std::vector<Pos> FindCorners(const Octave& img);

// Given a vector of {x,y,score} tuples, pre-sorted by 'y',
// supress any non-max tuples within a radius of 'k'.
// supression is done by setting the score to -1.
void SupressNonMax(int k, std::vector<faster::Corner>* corners);

#endif
