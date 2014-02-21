/*
 * octave.cpp
 *
 *  Created on: 04/02/2012
 *      Author: moreil
 */

#include <stdio.h>
#include <math.h>
#ifdef NEON
#include <arm_neon.h>
#endif
#include <glog/logging.h>

#include "util.h"
#include "octave.h"
#include "region.h"

Octave::Octave(const uint8_t* data, int width, int height) :
    space_(width, height), image_(new uint8_t[space_.size()]) {
  memcpy(image_.get(), data, space_.size());
}
Octave::Octave(const uint8_t* data, int width, int height, int stride) :
    space_(width, height, stride), image_(new uint8_t[space_.size()]) {
  memcpy(image_.get(), data, space_.size());
}
Octave::Octave(Octave&& other) : space_(other.space_), image_(std::move(other.image_)) {
  other.space_ = Space(0, 0);
}
Octave& Octave::operator=(Octave&& other) {
  if (this == &other) return *this;
  space_ = other.space_;
  image_ = std::move(other.image_);
  other.space_ = Space(0, 0);
  return *this;
}

Octave::~Octave() {}

Octave Octave::clone() const {
  return Octave(image_.get(), space_.width, space_.height, space_.stride);
}

const int Patch::kPatchRadius;
const int Patch::kPatchSize;
const uint8_t guass_weights[] = {
    4,   6,   9,  11,  11,   9,   6,   4,
    6,  11,  15,  18,  18,  15,  11,   6,
    9,  15,  21,  25,  25,  21,  15,   9,
   11,  18,  25,  30,  30,  25,  18,  11,
   11,  18,  25,  30,  30,  25,  18,  11,
    9,  15,  21,  25,  25,  21,  15,   9,
    6,  11,  15,  18,  18,  15,  11,   6,
    4,   6,   9,  11,  11,   9,   6,   4,
};

// Fill in this octave from the previous one.
// aka: Shrink the image from the previous octave to be half the width
// and half the height, and save the result into this octave.
void Octave::fill(const Octave& prev) {
  if (image_ == NULL || space_.width != prev.space_.width / 2 || space_.height != prev.space_.height/2) {
    space_ = Space(prev.space_.width / 2, prev.space_.height / 2);
    image_.reset(new uint8_t[space_.size()]);
  }

  for (int y = 0; y < space_.height;++y) {
    uint8_t* src1 = prev.pixel_ptr(Pos(0, 2 * y));
    uint8_t* src2 = prev.pixel_ptr(Pos(0, 2 * y + 1));
    uint8_t* dst = pixel_ptr(Pos(0, y));
    for (int x = 0; x < space_.width; ++x) {
      dst[x] = ((int)src1[x*2] + src1[x*2 + 1]+
          src2[x*2] + src2[x*2 + 1])/4;
    }
  }
}

#ifdef NEON
// Fixed size 8x8 scoring. uses neon intrinsics for some sembalance of speed.
inline int Octave::Score_neon(uint8_t* patch, Pos pos) const {

  uint8_t* ptr = pixel_ptr(pos - int(Patch::kPatchRadius));

  const uint8_t* guass_ptr = guass_weights;
  int step = space_.stride;
  uint16x8_t acc = vdupq_n_u16(0);

  for (int r = 8; r > 0 ; --r) {
    uint8x8_t img_row = vld1_u8(ptr);  // load
    uint8x8_t patch_row = vld1_u8(patch);  // load
    uint8x8_t delta = vabd_u8(img_row, patch_row); // abs diff
    uint8x8_t guass = vld1_u8(guass_ptr);   // load
    acc = vmlal_u8(acc, delta, guass); // acc+=delta*gauss

    ptr += step;
    patch += 8;
    guass_ptr += 8;
  }
  uint32x4_t a1 = vpaddlq_u16(acc);
  uint64x2_t a2 = vpaddlq_u32(a1);
  uint32x2_t a3 = vmovn_u64(a2);
  uint64x1_t a4 = vpaddl_u32(a3);

  return vget_lane_u64(a4, 0);
}

// Search around a radius of the current position in this octave for the
// best match for this patch.
// Search around a radius of the current position in this octave for the
// best match for this patch.
FPos Octave::searchPosition_neon(const FPos &fp,
    uint8_t* patch, int radius, int* bestptr) const {
  Pos p = pos(fp);

  Pos ll = clip_pos(p - radius, Patch::kPatchRadius);
  Pos ur = clip_pos(p + radius, Patch::kPatchRadius + 1);

  int best = (1<<30);
  Pos best_pos(-1, -1);

  uint8x8_t g0 = vld1_u8(guass_weights+0);
  uint8x8_t g1 = vld1_u8(guass_weights+8);
  uint8x8_t g2 = vld1_u8(guass_weights+16);
  uint8x8_t g3 = vld1_u8(guass_weights+24);
  uint8x8_t g4 = vld1_u8(guass_weights+32);
  uint8x8_t g5 = vld1_u8(guass_weights+40);
  uint8x8_t g6 = vld1_u8(guass_weights+48);
  uint8x8_t g7 = vld1_u8(guass_weights+56);

  uint8x8_t patch_row0 = vld1_u8(patch+0*8);
  uint8x8_t patch_row1 = vld1_u8(patch+1*8);
  uint8x8_t patch_row2 = vld1_u8(patch+2*8);
  uint8x8_t patch_row3 = vld1_u8(patch+3*8);
  uint8x8_t patch_row4 = vld1_u8(patch+4*8);
  uint8x8_t patch_row5 = vld1_u8(patch+5*8);
  uint8x8_t patch_row6 = vld1_u8(patch+6*8);
  uint8x8_t patch_row7 = vld1_u8(patch+7*8);

  int step = space_.stride;
  for (int py = ll.y ; py <= ur.y; ++py) {

    for (int px = ll.x ; px <= ur.x; ++px) {
      uint8_t* ptr = pixel_ptr(Pos(px,py) - kPatchRadius);

      uint8x8_t img_row0 = vld1_u8(ptr);
      ptr += step;
      uint8x8_t delta0 = vabd_u8(img_row0, patch_row0);
      uint16x8_t acc = vmull_u8(delta0, g0);
      uint8x8_t img_row1 = vld1_u8(ptr);

      ptr += step;
      uint8x8_t delta1 = vabd_u8(img_row1, patch_row1);
      uint8x8_t img_row2 = vld1_u8(ptr);
      acc = vmlal_u8(acc, delta1, g1);


      ptr += step;
      uint8x8_t delta2 = vabd_u8(img_row2, patch_row2);
      uint8x8_t img_row3 = vld1_u8(ptr);
      acc = vmlal_u8(acc, delta2, g2);


      ptr += step;
      uint8x8_t delta3 = vabd_u8(img_row3, patch_row3);
      uint8x8_t img_row4 = vld1_u8(ptr);
      acc = vmlal_u8(acc, delta3, g3);


      ptr += step;
      uint8x8_t delta4 = vabd_u8(img_row4, patch_row4);
      uint8x8_t img_row5 = vld1_u8(ptr);
      acc = vmlal_u8(acc, delta4, g4);


      ptr += step;
      uint8x8_t delta5 = vabd_u8(img_row5, patch_row5);
      uint8x8_t img_row6 = vld1_u8(ptr);
      acc = vmlal_u8(acc, delta5, g5);


      ptr += step;
      uint8x8_t delta6 = vabd_u8(img_row6, patch_row6);
      uint8x8_t img_row7 = vld1_u8(ptr);
      acc = vmlal_u8(acc, delta6, g6);

      uint8x8_t delta7 = vabd_u8(img_row7, patch_row7);
      acc = vmlal_u8(acc, delta7, g7);

      uint32x4_t a1 = vpaddlq_u16(acc);
      uint64x2_t a2 = vpaddlq_u32(a1);
      uint32x2_t a3 = vmovn_u64(a2);
      uint64x1_t a4 = vpaddl_u32(a3);

      int sum = vget_lane_u64(a4, 0);
      if (Pos(px, py) == p)  // Bias in favour of current position.
        sum--;
      if (sum > best)
        continue;
      best = sum;
      best_pos = Pos(px, py);
    }
  }
  //  LOG("Search moved from (%d,%d) to (%d,%d) sum %d\n",
      //      ix, iy, best_x, best_y, best);
  *bestptr = best;
  //return fp + octave.fpos(best_pos - p);
  return fpos(best_pos);
}



// Copy a patch from the given position in the octave.
void Octave::fillScale_neon(uint8_t *patch, const FPos& fpos) const {
  Pos p(pos(fpos));

  uint8_t* ptr = pixel_ptr(p - kPatchRadius);
  for (int ry = 0; ry < kPatchRadius*2; ++ry) {
    vst1_u8 (patch, vld1_u8(ptr));
    ptr += space_.stride;
    patch += 8;
  }
}


#endif


// Copy a patch from the given position in the octave.
Patch Octave::GetPatch(const FPos& fpos) const {
  Pos p(pos(fpos));
  Patch patch;
  uint8_t* patch_ptr = &patch.data[0];

  uint8_t* ptr = pixel_ptr(p - Patch::kPatchRadius);
  for (int ry = 0; ry < Patch::kPatchRadius * 2; ++ry) {
    for (int rx = 0 ; rx < Patch::kPatchRadius * 2; ++rx) {
      *patch_ptr++ = ptr[rx];
    }
    ptr += space_.stride;
  }
  return patch;
}

int Octave::Score(const Patch& patch, const Pos& pos) const {
  uint8_t* ptr = pixel_ptr(pos - Patch::kPatchRadius);
  const uint8_t* patch_ptr = &patch.data[0];

 const uint8_t* guass_ptr = guass_weights;
 int step = space_.stride;
 int acc = 0;

 for (int r = 8; r > 0 ; --r) {
   int16_t diff[8];
   for (int c = 0; c < 8; ++c)
     diff[c] = abs((int16_t)ptr[c] - patch_ptr[c]);
   for (int c = 0; c < 8; ++c)
     diff[c] = abs(diff[c]);
   for (int c = 0; c < 8; ++c)
     diff[c] = diff[c] * guass_ptr[c];
   for (int c = 0; c < 8; ++c)
     acc += diff[c];

   ptr += step;
   patch_ptr += 8;
   guass_ptr += 8;
 }

 return acc;
}

// Search around a radius of the current position in this octave for the
// best match for this patch.
FPos Octave::SearchPosition(const FPos &fp,
    const Patch& patch, int radius, int* bestptr) const {
  Pos p = pos(fp);

  Pos ll = clip_pos(p - radius, Patch::kPatchRadius);
  Pos ur = clip_pos(p + radius, Patch::kPatchRadius + 1);

  int best = (1<<30);
  Pos best_pos(Pos::invalid());
  for (auto& point : Region(ll, ur)) {
    int sum = Score(patch, point);
    if (point == p)
      sum--;
    if (sum > best)
      continue;
    best = sum;
    best_pos = point;
  }
  //  LOG("Search moved from (%d,%d) to (%d,%d) sum %d\n",
  //      ix, iy, best_x, best_y, best);
  *bestptr = best;
  //return fp + octave.fpos(best_pos - p);
  return fpos(best_pos);
}


// Search around a radius of the current position in this octave for the
// best match for this patch.
int Octave::ScorePosition(const FPos &fp,
    int radius) const {
  Patch patch(GetPatch(fp));

  Pos p = pos(fp);
  Pos ll = clip_pos(p - radius, Patch::kPatchRadius);
  Pos ur = clip_pos(p + radius, Patch::kPatchRadius + 1);

  int min_score = (1<<30);
  for (auto& point : Region(ll, ur)) {
    if (point == p)
      continue;
    int sum = Score(patch, point);
    if (sum > min_score)
      continue;
    min_score = sum;
  }
  return min_score;

}

// Score this point as a Harris corner.
// Measures a 7 x 7 patch.
int Octave::ScoreCorner(const Pos& pos) const {
  const uint8_t* ptr = pixel_ptr(pos - 2);
  int width = space_.stride;

  static const int weights[] = {
      1,2,4, 2,1,
      2,4,8, 4,2,
      4,8,16,8,4,
      2,4,8, 4,2,
      1,2,4, 2,1,
  };

  int gxx = 0;
  int gyy = 0;
  int gxy = 0;
  for (auto& point : Region(5,5)) {
    int p = point.y * width + point.x;
    int w = weights[point.x+point.y*5];
    int dxx = (int(ptr[p+1])-int(ptr[p-1]));
    int dyy = (int(ptr[p+width])-int(ptr[p-width]));

    gxx += w*dxx*dxx;
    gyy += w*dyy*dyy;
    gxy += w*dxx*dyy;
  }

  int64_t det = (int64_t)gxx*gyy-gxy*gxy;
  int64_t trace = (int64_t)gxx+gyy;

  // Harris corner.
  int64_t score = det - trace*trace/6;

  // KLT corner
  int64_t radical = trace*trace - 4 * det;
  int64_t eigen2 = (trace - sqrt(radical))/2;
  return eigen2;// / 65536LL;

 // LOG("%d,%d,%d : %lld / %lld = %lld => %lld", gxx, gyy, gxy,
 //     d, t*t, t*t/d, score);
  if (score < 0)
    score = 0;
  score /= 65536;

  return score;
}

void Octave::Smooth() {
  for (int loop = 0; loop < 3; ++loop) {
    Octave orig = clone();

    for (int y = 1; y < space_.height - 1; ++y) {
      for (int x = 1; x < space_.width - 1; ++x) {
        image_[space_.index(x, y)] = (int)
          ((int)orig.pixel(x-1,y-1)+orig.pixel(x,y-1)+orig.pixel(x+1,y-1)+
          orig.pixel(x-1,y+0)+orig.pixel(x,y+0)+orig.pixel(x+1,y+0)+
          orig.pixel(x-1,y+1)+orig.pixel(x,y+1)+orig.pixel(x+1,y+1)) / 9;
      }
    }
  }
}
