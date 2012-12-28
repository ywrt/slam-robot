/*
 * octaveset.h
 *
 *  Created on: Feb 18, 2012
 *      Author: michael
 */

#ifndef OCTAVESET_H_
#define OCTAVESET_H_
#include <stdint.h>

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
  static const int kSectors = 14;
  Octave* octave0_;
  Octave* octave1_;
  Octave* octave2_;
  Octave* octave3_;

  float pose_[16];
  char mask_[kSectors*kSectors];
  int search_x_;
  int search_y_;
public:
  OctaveSet();
  ~OctaveSet();

  void fill(uint8_t* data, int width, int height);

  void fillPatchSet(const FPos& fp, PatchSet* patch) const;

  FPos updatePosition(const OctaveSet& pimage, const FPos& pos, const FPos& ppos) const;
  FPos updatePosition(const PatchSet& ps, const FPos& pos) const;
  const float* pose() const { return pose_; };
  float* pose() { return pose_; };
  Pos pos0(const FPos& fp) const;


  FPos searchBestCorner(const FRegion& region) const;
  // While there's areas left to be searched, search for a corner.
  FPos find_next_corner();
  // Reset the first, and return the next corner.
  FPos find_first_corner();
  // Mask out areas of the integral image as known corners.
  void set_known_corner(const FPos& corner);
};


#endif /* OCTAVESET_H_ */
