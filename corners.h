#ifndef CORNERS_H_
#define CORNERS_H_

#include <vector>

class Octave;
namespace faster {
  class Corner;
}
class Region;

struct ScoredCorner {
  Pos pos;
  int score;
};

// An ordered, range searchable list of points.
struct CornerList {
  CornerList() {}
  CornerList(const std::vector<Pos>& corners);

  // Find the index of the first corner in the region r.
  // returns -1 if no such corner.
  int find(const Region& r) const;
  // Find the index of the next corner in the region r.
  // return -1 if no such corner. 
  int next(const Region& r, int index) const;

  std::vector<ScoredCorner> corners;
#ifndef CORNERS_H_
#define CORNERS_H_

#include <vector>

class Octave;
namespace faster {
  class Corner;
}
class Region;

struct ScoredCorner {
  Pos pos;
  int score;
};

// An ordered, range searchable list of points.
struct CornerList {
  CornerList() {}
  CornerList(const std::vector<Pos>& corners);

  // Find the index of the first corner in the region r.
  // returns -1 if no such corner.
  int find(const Region& r) const;
  // Find the index of the next corner in the region r.
  // return -1 if no such corner. 
  int next(const Region& r, int index) const;

  std::vector<ScoredCorner> corners;
};

// Find corners in an image with a threshold response of at least 'threshold',
// and then do non-max supression with a range of 'range'.
CornerList FindCorners(const Octave& img, int threshold, int range);

// Given a vector of {x,y,score} tuples, pre-sorted by 'y',
// supress any non-max tuples within a radius of 'k'.
// supression is done by setting the score to -1.
void SupressNonMax(int k, std::vector<faster::Corner>* corners);

#endif
};

// Find corners in an image with a threshold response of at least 'threshold',
// and then do non-max supression with a range of 'range'.
CornerList FindCorners(const Octave& img, int threshold, int range);

// Given a vector of {x,y,score} tuples, pre-sorted by 'y',
// supress any non-max tuples within a radius of 'k'.
// supression is done by setting the score to -1.
void SupressNonMax(int k, std::vector<faster::Corner>* corners);

#endif
