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
    fwd_hist { Histogram(20, 1000),
      Histogram(20, 1000),
      Histogram(20, 1000),
      Histogram(20, 1000),
    },
    rev_hist { Histogram(20, 1000),
      Histogram(20, 1000),
      Histogram(20, 1000),
      Histogram(20, 1000),
    } { }
OctaveSet::~OctaveSet() { }

void OctaveSet::FillOctaves(uint8_t* data, int width, int height) {
  octave0_.reset(new Octave(data, width, height)); // 2ms
  octave1_.reset(new Octave);
  octave2_.reset(new Octave);
  octave3_.reset(new Octave);
  octave1_->fill(*octave0_);  // 6 ms
  octave2_->fill(*octave1_);  // 1 ms
  octave3_->fill(*octave2_);  // 1 ms
}


Pos OctaveSet::pos0(const FPos& fp) const {
  return octave0_->pos(fp);
}

void OctaveSet::FillPatchSet(const FPos& fp, PatchSet* ps) const {
  ps->octave3 = octave3_->GetPatch(fp);
  ps->octave2 = octave2_->GetPatch(fp);
  ps->octave1 = octave1_->GetPatch(fp);
  ps->octave0 = octave0_->GetPatch(fp);
}

// 'pos' is the estimated position in the current OctaveSet needing
// refinement.
// 'ps' is a patchset to search for.
FPos OctaveSet::UpdatePosition(const PatchSet& ps,
                               const FPos& pos) const {
  FPos fp(pos);

  int suma;
  fp = octave3_->SearchPosition(fp, ps.octave3, 2, &suma);
  fp = octave2_->SearchPosition(fp, ps.octave2, 2, &suma);
  fp = octave1_->SearchPosition(fp, ps.octave1, 2, &suma);
  fp = octave0_->SearchPosition(fp, ps.octave0, 2, &suma);

  return fp;
}

// 'pos' is the estimated position in the current octaveset needing
// refinement.
// 'ppos' is the known position in the previous 'pimage' octaveset
FPos OctaveSet::UpdatePosition(const OctaveSet& pimage,
                               const FPos& pos, const FPos& ppos) const {
  Patch patch;
  FPos fp(pos);
  if (!pimage.octave3_->contains(ppos, Patch::kPatchRadius)) {
    return FPos::invalid();
  }
  int suma;
  patch = pimage.octave3_->GetPatch(ppos);
  fp = octave3_->SearchPosition(fp, patch, 2, &suma);
  fwd_hist[3].add(suma);
  patch = pimage.octave2_->GetPatch(ppos);
  fp = octave2_->SearchPosition(fp, patch, 2, &suma);
  fwd_hist[2].add(suma);
  patch = pimage.octave1_->GetPatch(ppos);
  fp = octave1_->SearchPosition(fp, patch, 2, &suma);
  fwd_hist[1].add(suma);
  patch = pimage.octave0_->GetPatch(ppos);
  fp = octave0_->SearchPosition(fp, patch, 2, &suma);
  fwd_hist[0].add(suma);
  // Now search in the previous image.

  int sumb;
  FPos rfp(ppos);  // Reverse position.
  if (!octave3_->contains(fp, Patch::kPatchRadius)) {
    return FPos::invalid();
  }
  patch = octave3_->GetPatch(fp);
  rfp = pimage.octave3_->SearchPosition(rfp, patch, 2, &sumb);
  rev_hist[3].add(sumb);
  patch = octave2_->GetPatch(fp);
  rfp = pimage.octave2_->SearchPosition(rfp, patch, 2, &sumb);
  rev_hist[2].add(sumb);
  patch = octave1_->GetPatch(fp);
  rfp = pimage.octave1_->SearchPosition(rfp, patch, 2, &sumb);
  rev_hist[1].add(sumb);
  patch = octave0_->GetPatch(fp);
  rfp = pimage.octave0_->SearchPosition(rfp, patch, 2, &sumb);
  rev_hist[0].add(sumb);

  Pos fwd = pimage.octave0_->pos(ppos);
  Pos rev = pimage.octave0_->pos(rfp);
  Pos delta(abs(fwd.x - rev.x), abs(fwd.y - rev.y));

  // TODO: Extract magic number.
  if ((delta.x + delta.y) > 1) {
    // Update failed.
#if 0
    LOG("UpdatePos failed: (%3d,%3d) -> (%3d,%3d) : (%-3d,%-3d) [%f,%f]\n",
        fwd.x, fwd.y, rev.x, rev.y,
        (fwd.x - rev.x), (fwd.y - rev.y),
        rfp.x, rfp.y);
#endif

    return FPos::invalid();
  }

  // Values computed from histograms.
  // TODO: Extract magic number.
  if (sumb > 18000 || suma > 18000) {
    return FPos::invalid();
  }

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

