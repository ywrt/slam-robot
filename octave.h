/*
 * octave.h
 *
 *  Created on: 04/02/2012
 *      Author: moreil
 */

#ifndef OCTAVE_H_
#define OCTAVE_H_
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "imgtypes.h"
#include "region.h"

extern const uint8_t guass_weights[];

class Octave {
public:
  static const int patch_radius = 4;
  static const int patch_size = (patch_radius*2) * (patch_radius*2);
  __restrict uint8_t* image_;
  int width_;
  int height_;

  Octave() : image_(NULL), width_(0), height_(0) {}

  // Fill in this octave from the previous one.
  // aka: Shrink the image from the previous octave to be half the width
  // and half the height, and save the result into this octave.
  void fill(const Octave& prev) {
    if (image_ == NULL || width_ != prev.width_ / 2 || height_ != prev.height_/2) {
      if (image_)
        free(image_);
      image_ = NULL;
      width_ = prev.width_/2;
      height_ = prev.height_/2;
      image_ = (uint8_t*) malloc(width_ * height_);
    }

    for (int y = 0; y < height_;++y) {
      uint8_t* src = prev.image_ + y*2*prev.width_;
      uint8_t* dst = image_ + y*width_;
      for (int x = 0; x < width_; ++x) {
        dst[x] = ((int)src[x*2]+src[x*2+1]+
            src[x*2+prev.width_]+src[x*2+prev.width_+1])/4;
      }
    }
  }

  void copy(uint8_t* image, int width, int height) {
    if (image_ == NULL || width_ != width || height_ != height) {
      if (image_)
        free(image_);
      image_ = NULL;
      width_ = width;
      height_ = height;
      image_ = (uint8_t*) malloc(width_ * height_);
    }
    memcpy(image_, image, width*height);
  }
  // Return the pixel at (x,y) from the image in this octave.
  // x,y in pixel co-ordinates.
  inline uint8_t pixel(int x, int y) const {
    return image_[y*width_+x];
  }
  inline uint8_t pixel(Pos p) const {
    return image_[p.y*width_+p.x];
  }
  inline uint8_t* pixel_ptr(Pos p) const {
    return &image_[p.y*width_+p.x];
  }

  // Convert floating-point position to
  // pixel position.
  inline Pos pos(const FPos& fp) const {
    int ix = fp.x * width_;
    int iy = fp.y * height_;
    return Pos(ix, iy);
  }
  // Convert pixel position to floating-point
  // position in [0,1]x[0,1]
  inline FPos fpos(const Pos& p) const {
    float x = (float(p.x)+0.01f) / width_;
    float y = (float(p.y)+0.01f) / height_;
    return FPos(x, y);
  }

  // Convert floating-point region to
  // pixel position.
  inline Region region(const FRegion& fp) const {
    return Region(pos(fp.ll), pos(fp.ur));
  }
  // Convert pixel position to floating-point
  // position in [0,1]x[0,1]
  inline FRegion fregion(const Region& fp) const {
    return FRegion(fpos(fp.ll), fpos(fp.ur));
  }

  // Clip a pixel position into the octave bounds.
  Pos clip_pos(const Pos& pos) const {
    Pos r = pos;
    if (r.x < 0) r.x = 0;
    if (r.y < 0) r.y = 0;
    if ((width_ - r.x) <= 0) r.x = width_-1;
    if ((height_ - r.y) <= 0) r.y = height_-1;
    return r;
  }
  // Clip a pixel position into the octave bounds.
  Pos clip_pos(const Pos& pos, int margin) const {
    Pos r = pos;
    if (r.x < margin) r.x = margin;
    if (r.y < margin) r.y = margin;
    if ((width_ - r.x) <= margin) r.x = width_-margin-1;
    if ((height_ - r.y) <= margin) r.y = height_-margin-1;
    return r;
  }

  inline Region clipped_region(
      const Region& reg,
      int margin) const {
    return Region(
        clip_pos(reg.ll, margin),
        clip_pos(reg.ur, margin));
  }
  inline Region clipped_region(
      const FRegion& freg,
      int margin) const {
    return clipped_region(region(freg), margin);
  }

  // Does this point line inside the clip region?
  inline bool contains_pos(const Pos& pos, int margin) const {
    Pos r = pos;
    if (r.x < margin) return false;
    if (r.y < margin) return false;
    if ((width_ - r.x) <= margin) return false;
    if ((height_ - r.y) <= margin) return false;
    return true;
  }
  inline bool contains_fpos(const FPos& fpos, int margin) const {
    return contains_pos(pos(fpos), margin);
  }

  // Compute the cross-correlation for a patch against
  // the given (ix,iy) position in the given octave.
  // ix, iy in pixel co-ords. Inputs assumed to be valid (i.e. not
  // outside the patch.
  int scorePosition(uint8_t* patch, int ix, int iy, int best) const {
    int sum = 0;
    for (int ry = iy - patch_radius ; ry < iy + patch_radius; ++ry) {
      for (int rx = ix - patch_radius ; rx < ix + patch_radius ; ++rx) {
        uint8_t a  = pixel(rx, ry);
        uint8_t b = *patch;
        int d = a - b;
        sum += abs(d);
        ++patch;
      }
    }
    return sum;
  }

  // Fixed size 8x8 scoring. uses neon intrinsics for some sembalance of speed.
  inline int Score_neon(uint8_t* patch, Pos pos) const;
  // Search around a radius of the current position in this octave for the
  // best match for this patch.
  // Search around a radius of the current position in this octave for the
  // best match for this patch.
  FPos searchPosition_neon(const FPos &fp,
      uint8_t* patch, int radius, int* bestptr) const;
  // Copy a patch from the given position in the octave.
  void fillScale(uint8_t *patch, const FPos& fpos) const;

  // Copy a patch from the given position in the octave.
  void fillScale_neon(uint8_t *patch, const FPos& fpos) const;

  int Score(const uint8_t* patch, const Pos& pos) const;
  // Search around a radius of the current position in this octave for the
  // best match for this patch.
  FPos searchPosition(const FPos &fp,
      const uint8_t* patch, int radius, int* bestptr) const;

  // Compute the harris score for the image at this position.
  int scoreCorner(const Pos& pos) const;
};


#endif /* OCTAVE_H_ */
