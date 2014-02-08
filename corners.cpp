#include <stdio.h>

#include "faster.h"
#include "imgtypes.h"
#include "octave.h"

using namespace std;

// Return scored corners. Corners are sorted by Y.
vector<faster::Corner> FindFasterCorners(const Octave& img, int k) {
  // Find corners, in order.
  vector<faster::Corner> corners = faster::faster_detect(img.image_, img.space_.width, img.space_.height, img.space_.stride, k);
  faster::faster_score(img.image_, img.space_.width, img.space_.height, img.space_.stride, k, &corners);

  return corners;
}

// Supress non-max corners. Done by setting score to -1 for every
// supressed corner.
void SupressNonMax(int k, vector<faster::Corner>* cptr) {
  vector<faster::Corner>& corners = *cptr;
  // Non-max supression over a k radius.

  // Search for corners that are not less than another other corner in 'k' distance.
  bool changes = false;
  int loops = 0;
  do {
    ++loops;
    changes = false;
    size_t left = 0;
    size_t right = 0;
    for (auto& c : corners) {
      if (c.score < 0)
        continue;
      //printf("[%d,%d]\n", c.x, c.y);
      // Adjust segment of corners to search. Corners are already
      // sorted by Y.
      while ((corners[left].y + k) < c.y && left < corners.size())
        ++left;
      while ((corners[right].y - k) <= c.y && right < corners.size())
        ++right;

      for (size_t i = left; i < right; ++i) {
        if (corners[i].score < 0)
          continue;
        if ((corners[i].x + k) < c.x)
          continue;
        if ((corners[i].x - k) > c.x)
          continue;
        if (corners[i].score > c.score)
          goto supressed;
      }

      // This corner is not supressed.
      // Supress all the covered corners.
      for (size_t i = left ; i < right; ++i) {
        if ((corners[i].x + k) < c.x)
          continue;
        if ((corners[i].x - k) > c.x)
          continue;
        // Don't supress the max corner itself.
        // TODO: if (corners[i].score < c.score) { ... }
        if (corners[i].x == c.x && corners[i].y == c.y)
          continue;

        //printf("c (%d,%d), corners[i] (%d,%d), k %d, left %d, right %d\n", c.x, c.y, corners[i].x, corners[i].y, k, left, right);
        if (corners[i].score >= 0) {
          corners[i].score = -1;
          changes = true;
        }
      }

     supressed: ;
    }
  } while (changes);
  //printf("Loops %d\n", loops);
}

vector<Pos> FindCorners(const Octave& img) {
  auto corners = FindFasterCorners(img, 5);
  for (const auto& c : corners) {
    if (c.score < 0)
      continue;
    //printf("%d,%d => %d\n", c.x, c.y, c.score);
  }

  SupressNonMax(15, &corners);

  vector<Pos> result;
  for (const auto& c : corners) {
    if (c.score < 0)
      continue;
    result.push_back(Pos(c.x, c.y));
  }
  return result;
}
