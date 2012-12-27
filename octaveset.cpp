/*
 * octaveset.cpp
 *
 *  Created on: Feb 18, 2012
 *      Author: michael
 */

#include <stdio.h>
#include "octave.h"
#include "util.h"

#include "octaveset.h"


OctaveSet::OctaveSet() :
    octave0_(new Octave()),
    octave1_(new Octave()),
    octave2_(new Octave()),
    octave3_(new Octave()),
    search_x_(0), search_y_(0) {
  memset(mask_, 0, sizeof(mask_));
}

OctaveSet::~OctaveSet() {
  delete octave3_;
  delete octave2_;
  delete octave1_;
  delete octave0_;
}

void OctaveSet::fill(uint8_t* data, int width, int height) {
  octave0_->copy((uint8_t*)data, width, height); // 2ms
  octave1_->fill(*octave0_);  // 6 ms
  octave2_->fill(*octave1_);  // 1 ms
  octave3_->fill(*octave2_);  // 1 ms
}


Pos OctaveSet::pos0(const FPos& fp) const {
  return octave0_->pos(fp);
}

// 'pos' is the estimated position in the current octaveset needing
// refinement.
// 'ppos' is the known position in the previous 'pimage' octaveset
FPos OctaveSet::updatePosition(const OctaveSet& pimage,
    const FPos& pos, const FPos& ppos) const {
  uint8_t patch[64];
  FPos fp(pos);
  if (!pimage.octave3_->contains_fpos(ppos, Octave::patch_radius)) {
    Pos p = octave3_->pos(ppos);
    LOG("Out of image fail %d,%d.\n", p.x, p.y);
    return FPos(-1, -1);
  }
  int suma;
  pimage.octave3_->fillScale(patch, ppos);
  fp = octave3_->searchPosition(fp, patch, 2, &suma);
  pimage.octave2_->fillScale(patch, ppos);
  fp = octave2_->searchPosition(fp, patch, 2, &suma);
  pimage.octave1_->fillScale(patch, ppos);
  fp = octave1_->searchPosition(fp, patch, 2, &suma);
  pimage.octave0_->fillScale(patch, ppos);
  fp = octave0_->searchPosition(fp, patch, 2, &suma);
  // Now search in the previous image.

  int sumb;
  FPos rfp(ppos);  // Reverse position.
  if (!octave3_->contains_fpos(fp, Octave::patch_radius)) {
    Pos p = octave3_->pos(fp);
    LOG("Out of image fail %d,%d.\n", p.x, p.y);
    return FPos(-1, -1);
  }
  octave3_->fillScale(patch, fp);
  rfp = pimage.octave3_->searchPosition(rfp, patch, 2, &sumb);
  octave2_->fillScale(patch, fp);
  rfp = pimage.octave2_->searchPosition(rfp, patch, 2, &sumb);
  octave1_->fillScale(patch, fp);
  rfp = pimage.octave1_->searchPosition(rfp, patch, 2, &sumb);
  octave0_->fillScale(patch, fp);
  rfp = pimage.octave0_->searchPosition(rfp, patch, 2, &sumb);

  Pos fwd = pimage.octave0_->pos(ppos);
  Pos rev = pimage.octave0_->pos(rfp);
  Pos delta(abs(fwd.x - rev.x), abs(fwd.y - rev.y));

  if ((delta.x + delta.y) > 13) {
    // Update failed.
#if 0
    LOG("updatePos failed: (%3d,%3d) -> (%3d,%3d) : (%-3d,%-3d) [%f,%f]\n",
        fwd.x, fwd.y, rev.x, rev.y,
        (fwd.x - rev.x), (fwd.y - rev.y),
        rfp.x, rfp.y);
#endif
    return FPos(-1,-1);
  }

  //LOG("sum %5d %5d : %d,%d\n", suma, sumb, delta.x, delta.y);
  return fp;
}



FPos OctaveSet::check_for_corner(int sx, int sy) const {
  FRegion freg(
      FPos(float(sx+1)/(kSectors+2), float(sy+1)/(kSectors+2)),
      FPos(float(sx+2)/(kSectors+2), float(sy+2)/(kSectors+2)));

  Region reg(octave3_->clipped_region(freg, 3));

  int min_score = 500000;
  int best_score = 0;
  Pos best_pos;
  for (int y = reg.ll.y; y < reg.ur.y; y ++) {
      for (int x = reg.ll.x; x < reg.ur.x; x ++) {
        int score = octave3_->scoreCorner(Pos(x,y));
        if (score < best_score)
          continue;
        best_score = score;
        best_pos = Pos(x,y);
      }
  }

  if (best_score < min_score)
    return FPos(-1,-1);
  min_score /= 2;

  // Use the factor that the octave are 2x apart.
  Pos ll(best_pos + best_pos - 1);
  Pos ur(best_pos + best_pos + 1);
  best_score = 0;
  for (int y = ll.y; y < ur.y; y ++) {
    for (int x = ll.x; x < ur.x; x ++) {
      int score = octave2_->scoreCorner(Pos(x,y));
      if (score < best_score)
        continue;
      best_score = score;
      best_pos = Pos(x,y);
    }
  }
  if (best_score < min_score)
    return FPos(-1,-1);
  min_score /= 2;

  // Use the factor that the octave are 2x apart.
  ll = Pos(best_pos + best_pos - 1);
  ur = Pos(best_pos + best_pos + 1);
  best_score = 0;
  for (int y = ll.y; y < ur.y; y ++) {
    for (int x = ll.x; x < ur.x; x ++) {
      int score = octave1_->scoreCorner(Pos(x,y));
      if (score < best_score)
        continue;
      best_score = score;
      best_pos = Pos(x,y);
    }
  }
  if (best_score < min_score)
    return FPos(-1,-1);
  min_score /= 2;


  // Use the factor that the octave are 2x apart.
  ll = Pos(best_pos + best_pos - 1);
  ur = Pos(best_pos + best_pos + 1);
  best_score = 0;
  for (int y = ll.y; y < ur.y; y ++) {
    for (int x = ll.x; x < ur.x; x ++) {
      int score = octave0_->scoreCorner(Pos(x,y));
      if (score < best_score)
        continue;
      best_score = score;
      best_pos = Pos(x,y);
    }
  }
  if (best_score < min_score)
    return FPos(-1,-1);
  min_score /= 2;

  //LOG("pos (%d,%d) => (%d,%d) => %d\n", sx, sy,
  //    best_pos.x, best_pos.y, best_score);

  FPos fp(octave0_->fpos(best_pos));
  return fp;
}


// Reset the first, and return the next corner.
FPos OctaveSet::find_first_corner() {
  search_x_ = 0;
  search_y_ = 0;
  return find_next_corner();
}

// While there's areas left to be searched, search for a corner.
FPos OctaveSet::find_next_corner() {
  FPos fp(-1,-1);
  while ( search_y_ < kSectors) {
    fp = check_for_corner(search_x_, search_y_);

    ++search_x_;
    if (search_x_ >= kSectors) {
      search_x_ = 0;
      ++search_y_;
    }
    if (fp.x != -1)
      return fp;
  }
  return FPos(-1,-1);
}

// Mask out areas of the integral image as known corners.
void OctaveSet::set_known_corner(const FPos& corner) {
  Pos p(corner.x * (kSectors+2), corner.y * (kSectors+2));
  if (p.x == 0 || p.y == 0)
    return;
  if (p.x >= kSectors || p.y >= kSectors)
    return;
  mask_[p.x + p.y*kSectors] = 1;
}
