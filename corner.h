/*
 * corner.h
 *
 *  Created on: 04/02/2012
 *      Author: moreil
 */

#ifndef CORNER_H_
#define CORNER_H_
#include <stdlib.h>
#include <Eigen/Core>

#include "imgtypes.h"

class OctaveSet;

class Corner {
  FPos fpos_;  // Position in current frame in [0,1]x[0,1]
  FPos ppos_;  // Previous position previous keyframe.
  int id_;     // Unique, ordered id.
  Eigen::Vector4f position_;  // Absolute position.
  static int next_id_;
public:
  Corner() : fpos_(-1.f,-1.f),
      ppos_(-1.f,-1.f), id_(next_id_++) { }

  void setPosition(const FPos& fpos) {
    // Check that it fits in smallest image.
    fpos_ = fpos;
  }
  int id() const { return id_; }
  inline const FPos& fpos() const { return fpos_; }
  inline const FPos& ppos() const { return ppos_; }
  inline bool invalid() const { return fpos_.x == -1; }
  inline void push_prev() { ppos_ = fpos_; }
};


#endif /* CORNER_H_ */
