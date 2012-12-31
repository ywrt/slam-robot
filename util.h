/*
 * util.h
 *
 *  Created on: 04/02/2012
 *      Author: moreil
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

#define LOG(fmt...) printf(fmt)

uint32_t get_usec();

uint64_t systemTime();

class Timer;
class ScopedTimer {
public:
  ScopedTimer(const char * name, int step = 255);
  ScopedTimer(Timer* timer);
  ~ScopedTimer();
private:
  Timer* timer_;
};



#endif /* UTIL_H_ */
