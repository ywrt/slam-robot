#include "octave.h"
#include "descriptor.h"
#include <iostream>
#include <stdio.h>

namespace {

Pos compare_list[] = {
  Pos(-8,0), Pos(-1, -3),
  Pos(-11,5), Pos(6, 5),
  Pos(12,-9), Pos(3, 6),
  Pos(-7,3), Pos(1, -1),
  Pos(0,11), Pos(2, 2),
  Pos(2,0), Pos(6, -2),
  Pos(6,9), Pos(-3, 0),
  Pos(-11,-7), Pos(5, -7),
  Pos(-3,8), Pos(6, -4),
  Pos(12,-12), Pos(5, -4),
  Pos(5,5), Pos(-4, -6),
  Pos(6,7), Pos(0, -3),
  Pos(6,10), Pos(7, -7),
  Pos(0,-2), Pos(-1, 1),
  Pos(12,8), Pos(0, -4),
  Pos(3,-13), Pos(-1, 0),
  Pos(-12,1), Pos(0, 3),
  Pos(3,0), Pos(-7, 3),
  Pos(-11,-3), Pos(5, 0),
  Pos(13,8), Pos(-7, 4),
  Pos(8,-2), Pos(-7, -1),
  Pos(-5,-9), Pos(-3, -7),
  Pos(4,8), Pos(5, -2),
  Pos(-11,7), Pos(7, 3),
  Pos(3,13), Pos(0, -7),
  Pos(-11,0), Pos(3, 1),
  Pos(-7,6), Pos(7, 3),
  Pos(6,-1), Pos(1, -2),
  Pos(12,5), Pos(1, -2),
  Pos(12,1), Pos(0, 5),
  Pos(7,-1), Pos(5, -7),
  Pos(10,-3), Pos(3, 7),
  Pos(-6,2), Pos(-3, -5),
  Pos(-5,-2), Pos(-7, 2),
  Pos(-6,-12), Pos(-7, 1),
  Pos(12,-4), Pos(7, 6),
  Pos(-5,10), Pos(2, 7),
  Pos(9,-8), Pos(-1, -6),
  Pos(2,13), Pos(2, 3),
  Pos(14,14), Pos(2, -7),
  Pos(1,-1), Pos(3, 5),
  Pos(-5,0), Pos(0, 7),
  Pos(-12,-10), Pos(4, 4),
  Pos(-8,-12), Pos(1, 4),
  Pos(12,12), Pos(-4, 0),
  Pos(4,-9), Pos(6, 7),
  Pos(14,-7), Pos(3, -2),
  Pos(0,11), Pos(-4, -6),
  Pos(4,13), Pos(3, 6),
  Pos(-1,-4), Pos(-5, 5),
  Pos(-10,-11), Pos(4, 4),
  Pos(9,13), Pos(0, 1),
  Pos(10,-12), Pos(6, 2),
  Pos(7,-12), Pos(-2, 5),
  Pos(4,-5), Pos(-6, -1),
  Pos(11,-4), Pos(2, 4),
  Pos(-2,0), Pos(-1, -5),
  Pos(-10,1), Pos(5, -5),
  Pos(-2,-6), Pos(-3, 1),
  Pos(9,2), Pos(0, 0),
  Pos(-3,-4), Pos(-4, 0),
  Pos(4,-3), Pos(-1, -6),
  Pos(-12,-1), Pos(-7, 5),
  Pos(-7,-11), Pos(-4, -5),
  Pos(-6,4), Pos(1, 6),
  Pos(10,-6), Pos(-4, 4),
  Pos(-6,11), Pos(-4, -3),
  Pos(8,2), Pos(7, 6),
  Pos(-11,6), Pos(-5, -5),
  Pos(14,3), Pos(7, -5),
  Pos(6,0), Pos(-4, 0),
  Pos(9,-9), Pos(6, 2),
  Pos(-12,7), Pos(-2, -5),
  Pos(-5,0), Pos(-2, 0),
  Pos(13,-8), Pos(-7, 2),
  Pos(-4,2), Pos(-5, 4),
  Pos(2,-9), Pos(2, 6),
  Pos(-12,0), Pos(4, -5),
  Pos(0,-9), Pos(-7, -3),
  Pos(-8,-1), Pos(-3, 1),
  Pos(13,1), Pos(1, -7),
  Pos(-8,12), Pos(5, 7),
  Pos(9,11), Pos(0, 3),
  Pos(-1,0), Pos(-6, -6),
  Pos(4,-10), Pos(4, -6),
  Pos(-13,12), Pos(0, -5),
  Pos(10,-3), Pos(5, -1),
  Pos(-5,4), Pos(-1, 2),
  Pos(-5,-11), Pos(-5, 4),
  Pos(-6,0), Pos(-4, -6),
  Pos(6,0), Pos(4, -7),
  Pos(12,-2), Pos(1, -1),
  Pos(10,-14), Pos(-6, 1),
  Pos(0,14), Pos(5, 4),
  Pos(-7,-4), Pos(-7, -3),
  Pos(-8,-12), Pos(0, -6),
  Pos(7,2), Pos(0, 2),
  Pos(-5,9), Pos(-1, -1),
  Pos(13,0), Pos(-6, 4),
  Pos(1,-9), Pos(-3, 4),
  Pos(-1,-10), Pos(-2, 1),
  Pos(7,-13), Pos(1, -4),
  Pos(10,-13), Pos(-6, 0),
  Pos(-8,7), Pos(-7, -7),
  Pos(13,-1), Pos(-4, 7),
  Pos(-8,14), Pos(7, -1),
  Pos(7,-9), Pos(3, 2),
  Pos(9,-11), Pos(3, 6),
  Pos(-12,0), Pos(-4, -1),
  Pos(10,0), Pos(-3, 0),
  Pos(4,0), Pos(2, 2),
  Pos(-6,-11), Pos(7, 4),
  Pos(9,-2), Pos(1, 1),
  Pos(1,-3), Pos(0, -6),
  Pos(-1,-11), Pos(7, 4),
  Pos(4,-4), Pos(-5, -2),
  Pos(-1,-8), Pos(3, -4),
  Pos(12,3), Pos(-3, 6),
  Pos(-9,11), Pos(-3, 6),
  Pos(0,0), Pos(1, 7),
  Pos(-5,-11), Pos(-7, 3),
  Pos(-7,-14), Pos(0, 1),
  Pos(-3,9), Pos(-5, -4),
  Pos(-13,11), Pos(0, -5),
  Pos(0,-14), Pos(5, -6),
  Pos(-8,-3), Pos(0, -1),
  Pos(-4,-10), Pos(-1, -6),
  Pos(-11,14), Pos(1, 7),
};

}

// Create a 128 measurement descriptor. Assumes image
// has already been smoothed.
Descriptor::Descriptor(const Octave& o, const Pos& pos) {
  // Take 8x16 == 128 random samples, concatentating the results.
  Pos* ptr = compare_list;
  for (int word = 0; word < 8; ++word) {
    uint32_t accum = 0;
    for (int i = 0; i < 16; ++i) {
      accum <<= 2;
      int a = o.pixel(pos + ptr[0]);
      int b = o.pixel(pos + ptr[1]);
      int delta = a - b;
      //std::cout << "delta " << delta << " a:" << a << " b:" << b <<"\n";
      ptr += 2;
      if (delta < -1)
        accum |= 2;
      else if (delta > 1)
        accum |= 1;
    }
    //printf("%2d %08x\n", word, accum);
    data[word] = accum;
  }

}

int Descriptor::distance(const uint32_t* vv) const {
  int bits = 0;
  for (int i = 0; i < 8; ++i) {
    uint32_t d = data[i] ^ vv[i];
    int count = __builtin_popcount(d);
    bits += count;
//    printf("%08x ^ %08x == %08x : %d (%d)\n", data[i], vv[i], d, count, bits);
  }
  return bits;
}
