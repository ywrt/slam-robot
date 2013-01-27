/*
 * histogram.cpp
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#include <iostream>
#include <iomanip>
#include <sstream>

#include "histogram.h"

using namespace std;

Histogram::Histogram(int buckets)
  : buckets_(buckets),
    scale_(1),
    counters_(buckets) { }
Histogram::Histogram(int buckets, double scale)
  : buckets_(buckets),
    scale_(scale),
    counters_(buckets) { }

void Histogram::add(double value) {
  int bucket = value / scale_;
  if (bucket >= buckets_)
    bucket = buckets_ - 1;
  if (bucket < 0)
    bucket = 0;
  counters_[bucket]++;
}

int Histogram::bucket(int n) const {
  return counters_[n];
}

std::string Histogram::str() const {
  std::ostringstream s;
  for (int i = 0; i < buckets_; ++i) {
    s << setw(6) << i * scale_ << ": " << counters_[i] << "\n";
  }
  return s.str();
}
