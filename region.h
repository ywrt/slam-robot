#ifndef REGION_H
#define REGION_H

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
// The region extents from [ll, ur] inclusive.
class Region {
public:
  Region(const Pos& tll, const Pos& tur) :
    ll(tll), ur(tur) {}
  Region(int width, int height) :
    ll(0,0), ur(width, height) {}
  Region() :
    ll(0,0), ur(-1, -1) {}

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

  Pos ll;
  Pos ur;
};

#endif
