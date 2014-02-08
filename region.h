#ifndef REGION_H_
#define REGION_H_

#include <stddef.h>

#include "imgtypes.h"

template<typename T>
struct RegionGeneric {
 T ll;
 T ur;

 RegionGeneric(const T& thell, const T& theur) :
   ll(thell), ur(theur) {}
};


typedef RegionGeneric<FPos> FRegion;

// An iterable region.
class Region {
public:
  // The region [ll, ur] inclusive.
  Region(const Pos& tll, const Pos& tur) :
    ll(tll), ur(tur) {}

  // A region of width x height.
  // Iterating yeilds the range [0, width) * [0, height)
  Region(int width, int height) :
    ll(0,0), ur(width - 1, height - 1) {}

  // A null region.
  Region() :
    ll(0,0), ur(-1, -1) {}

  bool contains(const Pos& p) const {
    return p.x >= ll.x &&
      p.x <= ur.x &&
      p.y >= ll.y &&
      p.y <= ur.y;
  }

  class iterator {
  public:
    iterator(const Region* region) :
      region_(region),
      pos_(region ? region->ll.x : 0,
        region ? region->ll.y : 0) {}
    iterator() : region_(NULL), pos_(0,0) {}

    const Pos& operator *() const { return pos_; }
    const iterator &operator ++() {
      pos_.x++;
      if (pos_.x > region_->ur.x) {
        pos_.x = region_->ll.x;
        pos_.y++;
        if (pos_.y > region_->ur.y) {
          region_ = NULL;
          pos_.x = pos_.y = 0;
        }
      }
      return *this;
    }
    iterator operator ++(int) {
      iterator copy(*this);
      ++(*this);
      return copy;
    }
    bool operator ==(const iterator &other) const {
      return region_ == other.region_ &&
          pos_ == other.pos_;
    }
    bool operator !=(const iterator &other) const {
      return !(*this == other);
    }

  private:
    const Region* region_;
    Pos pos_;
  };

  iterator begin() const {
    if (ll.x > ur.x || ll.y > ur.y)
      return iterator();
    return iterator(this);
  }
  iterator end() const {
    return iterator();
  }

  iterator begin() {
    if (ll.x > ur.x || ll.y > ur.y)
      return iterator();
    return iterator(this);
  }
  iterator end() {
    return iterator();
  }

  Pos ll;
  Pos ur;
};

#endif
