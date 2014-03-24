#ifndef IMAGE_H_
#define IMAGE_H_

#include <memory>

class CornerList;
class Octave;
class OctaveSet;

class Image {
 public:
  Image();
  ~Image();

  const Octave& octave(int n) const;
  const CornerList& corners() const;

  Patch GetPatch(const Pos& pos) const;
  Patch GetPatch(const FPos& fp) const;

 private:
  std::unique_ptr<Octave> octave_; 
  std::unique_ptr<OctaveSet> octave_; 

  std::unique_ptr<CornerList> fast_corners_; 
};

#endif
