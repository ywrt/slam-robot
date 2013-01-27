/*
 * octaveset.h
 *
 *  Created on: Feb 18, 2012
 *      Author: michael
 */

#ifndef OCTAVESET_H_
#define OCTAVESET_H_
#include <stdint.h>
#include "imgtypes.h"
#include "region.h"
#include "histogram.h"

class Octave;

struct Patch {
  uint8_t data[64];
};

struct PatchSet {
  Patch octave0;
  Patch octave1;
  Patch octave2;
  Patch octave3;
};


class OctaveSet {
 public:
  OctaveSet();
  ~OctaveSet();

  void FillOctaves(uint8_t* data, int width, int height);

  void FillPatchSet(const FPos& fp, PatchSet* patch) const;

  FPos UpdatePosition(const OctaveSet& pimage, const FPos& pos, const FPos& ppos) const;
  FPos UpdatePosition(const PatchSet& ps, const FPos& pos) const;
  const float* pose() const { return pose_; };
  float* pose() { return pose_; };
  Pos pos0(const FPos& fp) const;


  FPos SearchBestCorner(const FRegion& region, int min_score) const;

  int CheckCorner(const FPos& pos);

  mutable Histogram fwd_hist[4];
  mutable Histogram rev_hist[4];

 private:
  Octave* octave0_;
  Octave* octave1_;
  Octave* octave2_;
  Octave* octave3_;

  float pose_[16];
};


#endif /* OCTAVESET_H_ */
