#ifndef FASTER_H_
#define FASTER_H_
#include <vector>
#include <stdint.h>

namespace faster {

struct Corner {
  int x;
  int y;
  int score;
};

void faster_score(uint8_t* bytes, int width, int height, int stride, int b,
    std::vector<Corner>* corners);

std::vector<Corner> faster_detect(uint8_t* bytes, int width, int height, int stride, int b);

}  // namespace

#endif
