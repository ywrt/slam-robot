#ifndef FASTER_H_
#define FASTER_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Corner {
  int x;
  int y;
  int score;
};

void faster_score(uint8_t* bytes, int width, int height, int stride, int b, struct Corner* corners, int num_corners);

int faster_detect(uint8_t* bytes, int width, int height, int stride, int b, struct Corner** corners);

#ifdef __cplusplus
}  // extern
#endif

#endif
