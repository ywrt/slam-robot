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

struct Patch {
  uint8_t data[64];
};

class Octave {
public:
  static const int patch_radius = 4;
  static const int patch_size = (patch_radius*2) * (patch_radius*2);

  __restrict uint8_t* image_;
  Space space_;

  Octave() : image_(NULL), space_(0,0) {}
  Octave(const Octave& src) :
      image_((uint8_t*)malloc(src.space_.size())),
      space_(src.space_) {
    memcpy(image_, src.image_, space_.size());
  }
  ~Octave() {
    free(image_);
    image_ = NULL;
  }

  int width() const { return space_.width; }
  int height() const { return space_.height; }
  const Space& space() const { return space_; }

  // Fill in this octave from the previous one.
  // aka: Shrink the image from the previous octave to be half the width
  // and half the height, and save the result into this octave.
  void fill(const Octave& prev);

  void copy(uint8_t* image, int width, int height);

  // Return the pixel at (x,y) from the image in this octave.
  // x,y in pixel co-ordinates.
  inline uint8_t pixel(int x, int y) const {
    return image_[space_.index(Pos(x, y))];
  }
  inline uint8_t pixel(Pos p) const {
    return image_[space_.index(p)];
  }
  inline uint8_t* pixel_ptr(Pos p) const {
    return &image_[space_.index(p)];
  }

  // Convert floating-point position to
  // pixel position.
  inline Pos pos(const FPos& fp) const { return space_.convert(fp); }
  // Convert pixel position to floating-point
  // position in [0,1]x[0,1]
  inline FPos fpos(const Pos& p) const { return space_.convert(p); }

  // Clip a pixel position into the octave bounds.
  Pos clip_pos(const Pos& pos) const { return space_.clip(pos); }

  // Clip a pixel position into the octave bounds.
  Pos clip_pos(const Pos& pos, int margin) const { return space_.clip(pos, margin); }

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
    return clipped_region(
        Region(
            space_.convert(freg.ll),
            space_.convert(freg.ur)),
            margin);
  }

  // Does this point line inside the clip region?
  inline bool contains(const Pos& pos, int margin) const { return space_.contains(pos, margin); }
  inline bool contains(const FPos& fpos, int margin) const { return contains(pos(fpos), margin); }

  // Fixed size 8x8 scoring. uses neon intrinsics for some sembalance of speed.
  inline int Score_neon(uint8_t* patch, Pos pos) const;

  // Search around a radius of the current position in this octave for the
  // best match for this patch.
  // Search around a radius of the current position in this octave for the
  // best match for this patch.
  FPos searchPosition_neon(const FPos &fp,
      uint8_t* patch, int radius, int* bestptr) const;
  // Copy a patch from the given position in the octave.
  // Copy a patch from the given position in the octave.
  void fillScale_neon(uint8_t *patch, const FPos& fpos) const;


  Patch GetPatch(const FPos& fpos) const;

  // Measure the disparity between the supplied patch, and the
  // image as position 'pos'.
  int Score(const Patch& patch, const Pos& pos) const;

  // Search around a radius of the current position in this octave for the
  // best match for this patch.
  FPos SearchPosition(const FPos &fp, const Patch& patch,
      int radius, int* bestptr) const;

  // Search around a radius of the current position in this octave for the
  // best match of the patch at the current positions. Good matches are
  // considered poor corners (i.e. not highly position specific).
  int ScorePosition(const FPos &fp,
                      int radius) const;

  // Compute the harris score for the image at this position.
  int ScoreCorner(const Pos& pos) const;

  // smooth in-place with a 3x3 box filter, repeated 3 times.
  void Smooth();
};


#endif /* OCTAVE_H_ */
