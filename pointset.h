#ifndef POINTSET_H_
#define POINTSET_H_

// A set holding points that permits 2-d range queries.
// typename T must have a constructor taking 'Pos'.

#include <set>

template<typename T>
class PointSet {
 public:
  PointSet() {}
  template<class InputIterator>
    PointSet(InputIterator first, InputIterator last) : points_(first, last) { }

  vector<T> intersect(const T& ll, const T& ur) const {
    vector<T> result;

    auto iter = points_.lower_bound(ll);
    
    
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

 private:
  std::set<T> points_;
};

#endif
