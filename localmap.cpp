/*
 * localmap.cpp
 *
 *  Created on: Dec 31, 2012
 *      Author: michael
 */

#include <glog/logging.h>

#include "localmap.h"

int LocalMap::AddFrame() {
  int frame_num = frames.size();

  if (frames.size() > 3) {
    auto s = frames.size();
    auto& f1 = frames[s - 1];
    auto& f2 = frames[s - 2];
    Vector3d motion = f1.translation_ - f2.translation_;
    if (motion.norm() > 1)
      motion /= motion.norm();
    Frame f = f1;
    f.translation_ += motion;
    frames.push_back(f);
  } else if (frames.size() > 0) {
    frames.push_back(frames.back());
    frames.back().translation()[0] += 0.01;
  } else {
    frames.push_back(Frame());
  }

  return frame_num;
}

 void LocalMap::Clean() {
   // SortObs sorter;
   // sort(map->obs.begin(), map->obs.end(), sorter);
   int err_hist[20] = {0,0,0,0,0,0,0,0,0,0};

   int curr_frame = frames.size() - 1;
   for (auto& point : points) {
     point.location_.normalize();
     if (point.num_observations() < 2)
       continue;
     int poor_matches = 0;
     for (auto& o : point.observations_) {
       double err = o.error.norm() * 1000;
       if (err < sizeof(err_hist) / sizeof(err_hist[0])) {
         ++err_hist[(int)err];
       }

       if (err < 5)
         continue;
       printf("frame %3d : (matches %d) [%7.3f %7.3f] (%7.2f,%7.2f) -> %.2f\n",
           o.frame_ref,
           point.num_observations(),
           o.pt(0), o.pt(1),
           o.error(0) * 1000, o.error(1) * 1000,
           err);
       ++poor_matches;
     }
     if (poor_matches && point.last_frame() == curr_frame) {
       point.bad_ = true;
       point.observations_.pop_back();
     }
   }
   for (size_t i = 0; i < sizeof(err_hist) / sizeof(err_hist[0]); ++i) {
     printf("err_hist: %2ld : %5d\n", i, err_hist[i]);
   }
 }
