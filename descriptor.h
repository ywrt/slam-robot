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
  Descriptor() : data{0,0,0,0, 0,0,0,0} {}

  int distance(const uint32_t* vv) const;

  int score() const {
    int bits = 0;
    for (const auto& v : data) {
      bits += __builtin_popcount(v);
    }
    return bits;
  }

  int distance(const Descriptor& desc) const {
    return distance(desc.data);
  }

  uint32_t data[8];
};



#endif
