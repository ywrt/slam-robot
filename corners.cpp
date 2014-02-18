#include <stdio.h>
#include <algorithm>

#include "faster.h"
#include "imgtypes.h"
#include "octave.h"
#include "corners.h"

using namespace std;

// Return scored corners. Corners are sorted by Y.
vector<ScoredCorner> FindFasterCorners(const Octave& img, int k) {
  // Find corners, in order.
  Corner* corners;
  int num_corners = faster_detect((uint8_t*)img.data(), img.space().width, img.space().height, img.space().stride, k, &corners);
  faster_score((uint8_t*)img.data(), img.space().width, img.space().height, img.space().stride, k, corners, num_corners);

  vector<ScoredCorner> result;
  for (int i = 0; i < num_corners; ++i)
    result.push_back({{corners[i].x, corners[i].y}, corners[i].score});
  
  free(corners);

  return result;
}

// Supress non-max corners. Done by setting score to -1 for every
// supressed corner.
void SupressNonMax(int k, vector<ScoredCorner>* cptr) {
  vector<ScoredCorner>& corners = *cptr;
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
      while ((corners[left].pos.y + k) < c.pos.y && left < corners.size())
        ++left;
      while ((corners[right].pos.y - k) <= c.pos.y && right < corners.size())
        ++right;

      for (size_t i = left; i < right; ++i) {
        if (corners[i].score < 0)
          continue;
        if ((corners[i].pos.x + k) < c.pos.x)
          continue;
        if ((corners[i].pos.x - k) > c.pos.x)
          continue;
        if (corners[i].score > c.score)
          goto supressed;
      }

      // This corner is not supressed.
      // Supress all the covered corners.
      for (size_t i = left ; i < right; ++i) {
        if ((corners[i].pos.x + k) < c.pos.x)
          continue;
        if ((corners[i].pos.x - k) > c.pos.x)
          continue;
        // Don't supress the max corner itself.
        // TODO: if (corners[i].score < c.score) { ... }
        if (corners[i].pos.x == c.pos.x && corners[i].pos.y == c.pos.y)
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

CornerList FindCorners(const Octave& img, int threshold, int range) {
  auto corners = FindFasterCorners(img, threshold);

  SupressNonMax(range, &corners);

  CornerList result;
  for (const auto& c : corners) {
    if (c.score < 0)
      continue;
    result.corners.push_back(c);
  }
  return result;
}

CornerList::CornerList(const std::vector<Pos>& c) {
  for (const auto& p : c)
    corners.push_back({p, 0});
  std::sort(corners.begin(),
      corners.end(),
      [](const ScoredCorner& a, const ScoredCorner& b) -> bool {
        if (a.pos.y != b.pos.y)
          return a.pos.y < b.pos.y;
        return a.pos.x < b.pos.x;
      });
}

int CornerList::find(const Region& r) const {
  return next(r, -1);
}

// Find first corner that is in the region.
// We use then lowest y, and then the lowest x.
// We start from the first corner after 'index'.
int CornerList::next(const Region& r, int index) const {
  ++index;
  if ((size_t)index == corners.size())
    return -1;  // End of corner list.

  while (1) {
    // If index is in the region, we're done.
    const Pos& p = corners[index].pos;
    if (r.contains(p))
      return index;

    // If we're to the left of the region, we search
    // along the current line for the beginning of the region.
    // otherwise we search from the next line.
    Pos corner;
    if (p.x < r.ll.x)
      corner = Pos(r.ll.x, p.y);
    else
      corner = Pos(r.ll.x, p.y + 1);

    int left = index;
    int right = corners.size() - 1;

    while (1) {
      int mid = (left + right) / 2;
      const Pos& p = corners[mid].pos;
      if (p < corner) {
        left = mid + 1;
      } else {
        right = mid;
      }
      if (left >= right)
        break;
    }
    if (left > right || corners[left].pos > r.ur)
      return -1;

    index = left;
  }
}
