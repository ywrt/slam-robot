/*
 * histogram.h
 *
 *  Created on: Jan 27, 2013
 *      Author: michael
 */

#ifndef HISTOGRAM_H_
#define HISTOGRAM_H_

#include <memory>
#include <string>
#include <vector>

class Histogram {
 public:
  Histogram(int buckets);
  Histogram(int buckets, double scale);

  void add(double value);
  int bucket(int n) const;

  std::string str() const;
 private:
  int buckets_;
  double scale_;
  std::vector<int> counters_;
};

#endif /* HISTOGRAM_H_ */
