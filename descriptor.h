#ifndef DESCRIPTOR_H_
#define DESCRIPTOR_H_

class Octave;
class Pos;

// A BRIEF-base feature descriptor.
// Uses 2 bits per comparison:
//   10 if comparison was less than 'a'
//   00 if comparison is in [-a, a]
//   01 if comparison was greater than 'a'.
// descriptor are 256 bits long (128 measurements).
struct Descriptor {
  Descriptor(const Octave& octave, const Pos& pos);

  int distance(const uint32_t* vv) const {
    int bits = 0;
    for (int i = 0; i < 8; ++i) {
      uint32_t d = data[i] ^ vv[i];
      int count = __builtin_popcount(d);
      bits += count;
    }
    return bits;
  }

  int distance(const Descriptor& desc) const {
    return distance(desc.data);
  }

  uint32_t data[8];
};



#endif
