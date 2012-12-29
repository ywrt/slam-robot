/*
 * grid.h
 *
 *  Created on: Dec 29, 2012
 *      Author: michael
 */

#ifndef GRID_H_
#define GRID_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "region.h"

class Grid {
 public:
  Grid(int width, int height)
     : width_(width),
       height_(height),
       data_(new uint8_t[width*height]) {
    memset(data_, 0, width_ * height_);
  }

  virtual ~Grid() {
    delete[] data_;
  }

  uint8_t* ptr(int x, int y) const {
    if (x < 0 ||y < 0)
      return NULL;
    if (x >= width_ || y >= width_)
      return NULL;
    return data_ + y * width_ + x;
  }

  // Mark a grid sector.
  // Returns the number of grid sectors newly marked.
  int mark(int x, int y) {
    auto p = ptr(x, y);
    if (p == NULL)
      return 0;
    auto prev = *p;
    *p = 1;
    if (!prev)
      return 1;
    return 0;
  }

  // Mark a grid sector and its 8 neighbors.
  // Returns the number of grid sectors newly marked.
  int groupmark(int x, int y) {
    int sum = 0;
    for (auto& p : Region(Pos(x-1, y-1), Pos(x+1, y+1))) {
      sum += mark(p.x, p.y);
    }
    return sum;
  }

  // Return true if this sector is not marked.
  bool isAvailable(int x, int y) const {
    auto p = ptr(x, y);
    if (!p)
      return false;
    return (*p == 0);  // Available if not marked.
  }

  // Return a region matching the grid sectors.
  Region region() const {
    return Region(width_ - 1, height_ - 1);
  }

  // Iterate over the unmarked grid sectors.
  class iterator {
   public:
    iterator() : grid_(NULL) { }
    iterator(const Grid* grid) :
      grid_(grid),
      region_(grid->region()),
      iter_(region_.begin()) {
      while (iter_ != region_.end() &&
          !grid_->isAvailable((*iter_).x, (*iter_).y)) {
        iter_++;
      }
    }

    const Pos& operator *() const { return *iter_; }
    const iterator &operator ++() {
      do {
        iter_++;
      } while (iter_ != region_.end() &&
          !grid_->isAvailable((*iter_).x, (*iter_).y));
      return *this;
    }
    iterator operator ++(int) {
      iterator copy(*this);
      ++(*this);
      return copy;
    }
    bool operator ==(const iterator &other) const {
      return iter_ == other.iter_;
    }
    bool operator !=(const iterator &other) const {
      return !(*this == other);
    }

   private:
    const Grid* grid_;
    Region region_;
    Region::iterator iter_;
  };

  iterator begin() const {
    return iterator(this);
  }
  iterator end() const {
    return iterator();
  }

  int width_;
  int height_;
  uint8_t* data_;
};

#endif /* GRID_H_ */
