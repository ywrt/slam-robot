/*
 * octave.cpp
 *
 *  Created on: 04/02/2012
 *      Author: moreil
 */

#ifdef NEON
#include <arm_neon.h>
#endif
#include "util.h"
#include "octave.h"
#include <stdio.h>

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

#ifdef NEON
// Fixed size 8x8 scoring. uses neon intrinsics for some sembalance of speed.
inline int Octave::Score_neon(uint8_t* patch, Pos pos) const {
  uint8_t* ptr = pixel_ptr(pos - int(patch_radius));

  const uint8_t* guass_ptr = guass_weights;
  int step = width_;
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

  Pos ll = clip_pos(p - radius, patch_radius);
  Pos ur = clip_pos(p + radius, patch_radius+1);

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

  int step = width_;
  for (int py = ll.y ; py <= ur.y; ++py) {

    for (int px = ll.x ; px <= ur.x; ++px) {
      uint8_t* ptr = pixel_ptr(Pos(px,py) - patch_radius);

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

  uint8_t* ptr = pixel_ptr(p - patch_radius);
  for (int ry = 0; ry < patch_radius*2; ++ry) {
    vst1_u8 (patch, vld1_u8(ptr));
    ptr += width_;
    patch += 8;
  }
}


#endif


// Copy a patch from the given position in the octave.
void Octave::fillScale(uint8_t *patch, const FPos& fpos) const {
  Pos p(pos(fpos));

  uint8_t* ptr = pixel_ptr(p - patch_radius);
  for (int ry = 0; ry < patch_radius*2; ++ry) {
    ptr += width_;
    for (int rx = 0 ; rx < patch_radius*2 ; ++rx) {
      *patch++ = ptr[rx];
    }
  }
}

int Octave::Score(const uint8_t* patch, const Pos& pos) const {
  uint8_t* ptr = pixel_ptr(pos - int(patch_radius));

   const uint8_t* guass_ptr = guass_weights;
   int step = width_;
   int acc = 0;

   for (int r = 8; r > 0 ; --r) {
     int16_t diff[8];
     for (int c = 0; c < 8; ++c)
       diff[c] = abs((int16_t)ptr[c] - patch[c]);
     for (int c = 0; c < 8; ++c)
       diff[c] = abs(diff[c]);
     for (int c = 0; c < 8; ++c)
       diff[c] = diff[c] * guass_ptr[c];
     for (int c = 0; c < 8; ++c)
       acc += diff[c];

     ptr += step;
     patch += 8;
     guass_ptr += 8;
   }

   return acc;
}


// Search around a radius of the current position in this octave for the
// best match for this patch.
FPos Octave::searchPosition(const FPos &fp,
    const uint8_t* patch, int radius, int* bestptr) const {
  Pos p = pos(fp);

  Pos ll = clip_pos(p - radius, patch_radius);
  Pos ur = clip_pos(p + radius, patch_radius+1);

  int best = (1<<30);
  Pos best_pos(Pos::invalid());
  for (auto& point : Region(ll, ur)) {
    //int sum = scorePosition(octave, patch, px, py, best);
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

// Score this point as a Harris corner.
int Octave::scoreCorner(const Pos& pos) const {
  const uint8_t* ptr = pixel_ptr(pos - 2);
  int width = width_;

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

  int64_t d = (int64_t)gxx*gyy-gxy*gxy;
  int64_t t = (int64_t)gxx+gyy;

  int64_t score = d - t*t/6;

 // LOG("%d,%d,%d : %lld / %lld = %lld => %lld", gxx, gyy, gxy,
 //     d, t*t, t*t/d, score);
  if (score < 0)
    score = 0;
  score /= 65536;

  return score;
}



