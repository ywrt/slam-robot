/*
 * octaveset.cpp
 *
 *  Created on: Feb 18, 2012
 *      Author: michael
 */

#include <stdio.h>
#include <algorithm>

#include "octave.h"
#include "util.h"

#include "octaveset.h"


OctaveSet::OctaveSet() :
octave0_(new Octave()),
octave1_(new Octave()),
octave2_(new Octave()),
octave3_(new Octave())
{}

OctaveSet::~OctaveSet() {
  delete octave3_;
  delete octave2_;
  delete octave1_;
  delete octave0_;
}

void OctaveSet::FillOctaves(uint8_t* data, int width, int height) {
  octave0_->copy((uint8_t*)data, width, height); // 2ms
  octave1_->fill(*octave0_);  // 6 ms
  octave2_->fill(*octave1_);  // 1 ms
  octave3_->fill(*octave2_);  // 1 ms
}


Pos OctaveSet::pos0(const FPos& fp) const {
  return octave0_->pos(fp);
}

void OctaveSet::FillPatchSet(const FPos& fp, PatchSet* ps) const {
  octave3_->FillPatch(ps->octave3.data, fp);
  octave2_->FillPatch(ps->octave2.data, fp);
  octave1_->FillPatch(ps->octave1.data, fp);
  octave0_->FillPatch(ps->octave0.data, fp);
}

// 'pos' is the estimated position in the current OctaveSet needing
// refinement.
// 'ps' is a patchset to search for.
FPos OctaveSet::UpdatePosition(const PatchSet& ps,
                               const FPos& pos) const {
  FPos fp(pos);

  int suma;
  fp = octave3_->SearchPosition(fp, ps.octave3.data, 2, &suma);
  fp = octave2_->SearchPosition(fp, ps.octave2.data, 2, &suma);
  fp = octave1_->SearchPosition(fp, ps.octave1.data, 2, &suma);
  fp = octave0_->SearchPosition(fp, ps.octave0.data, 2, &suma);

  return fp;
}


// 'pos' is the estimated position in the current octaveset needing
// refinement.
// 'ppos' is the known position in the previous 'pimage' octaveset
FPos OctaveSet::UpdatePosition(const OctaveSet& pimage,
                               const FPos& pos, const FPos& ppos) const {
  uint8_t patch[64];
  FPos fp(pos);
  if (!pimage.octave3_->contains_fpos(ppos, Octave::patch_radius)) {
    Pos p = octave3_->pos(ppos);
    LOG("Out of image fail %d,%d.\n", p.x, p.y);
    return FPos::invalid();
  }
  int suma;
  pimage.octave3_->FillPatch(patch, ppos);
  fp = octave3_->SearchPosition(fp, patch, 2, &suma);
  pimage.octave2_->FillPatch(patch, ppos);
  fp = octave2_->SearchPosition(fp, patch, 2, &suma);
  pimage.octave1_->FillPatch(patch, ppos);
  fp = octave1_->SearchPosition(fp, patch, 2, &suma);
  pimage.octave0_->FillPatch(patch, ppos);
  fp = octave0_->SearchPosition(fp, patch, 2, &suma);
  // Now search in the previous image.

  int sumb;
  FPos rfp(ppos);  // Reverse position.
  if (!octave3_->contains_fpos(fp, Octave::patch_radius)) {
    Pos p = octave3_->pos(fp);
    LOG("Out of image fail %d,%d.\n", p.x, p.y);
    return FPos::invalid();
  }
  octave3_->FillPatch(patch, fp);
  rfp = pimage.octave3_->SearchPosition(rfp, patch, 2, &sumb);
  octave2_->FillPatch(patch, fp);
  rfp = pimage.octave2_->SearchPosition(rfp, patch, 2, &sumb);
  octave1_->FillPatch(patch, fp);
  rfp = pimage.octave1_->SearchPosition(rfp, patch, 2, &sumb);
  octave0_->FillPatch(patch, fp);
  rfp = pimage.octave0_->SearchPosition(rfp, patch, 2, &sumb);

  Pos fwd = pimage.octave0_->pos(ppos);
  Pos rev = pimage.octave0_->pos(rfp);
  Pos delta(abs(fwd.x - rev.x), abs(fwd.y - rev.y));

  if ((delta.x + delta.y) > 2) {
    // Update failed.
    LOG("updatePos failed: (%3d,%3d) -> (%3d,%3d) : (%-3d,%-3d) [%f,%f]\n",
        fwd.x, fwd.y, rev.x, rev.y,
        (fwd.x - rev.x), (fwd.y - rev.y),
        rfp.x, rfp.y);

    return FPos::invalid();
  }

  //LOG("sum %5d %5d : %d,%d\n", suma, sumb, delta.x, delta.y);
  return fp;
}



FPos OctaveSet::SearchBestCorner(const FRegion& freg, int min_score) const {
  Region reg(octave3_->clipped_region(freg, 3));

  int best_score = 0;
  Pos best_pos;
  for (auto& p : reg) {
    int score = octave3_->ScoreCorner(p);
    if (score < best_score)
      continue;
    best_score = score;
    best_pos = p;
  }

  if (best_score < min_score)
    return FPos::invalid();
  min_score /= 2;

  // Use the factor that the octave are 2x apart.
  best_score = 0;
  for (auto& p : Region(best_pos * 2 - 1, best_pos * 2 + 1)) {
    int score = octave2_->ScoreCorner(p);
    if (score < best_score)
      continue;
    best_score = score;
    best_pos = p;
  }
  if (best_score < min_score)
    return FPos::invalid();
  min_score /= 2;

  // Use the factor that the octave are 2x apart.
  best_score = 0;
  for (auto& p : Region(best_pos * 2 - 1, best_pos * 2 + 1)) {
    int score = octave1_->ScoreCorner(p);
    if (score < best_score)
      continue;
    best_score = score;
    best_pos = p;

  }
  if (best_score < min_score)
    return FPos::invalid();
  min_score /= 2;

  best_score = 0;
  for (auto& p : Region(best_pos * 2 - 1, best_pos * 2 + 1)) {
    int score = octave0_->ScoreCorner(p);
    if (score < best_score)
      continue;
    best_score = score;
    best_pos = p;

  }
  if (best_score < min_score)
    return FPos::invalid();


  // LOG("(%d,%d) => %d\n",
  //best_pos.x, best_pos.y, best_score);

  FPos fp(octave0_->fpos(best_pos));
  return fp;
}

int OctaveSet::CheckCorner(const FPos& fp) {

  int score = octave3_->ScorePosition(fp, 5);
  score = std::min(score, octave2_->ScorePosition(fp, 5));
  score = std::min(score, octave1_->ScorePosition(fp, 5));
  score = std::min(score, octave0_->ScorePosition(fp, 5));

  return score;
}

