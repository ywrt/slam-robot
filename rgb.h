#ifndef RGB_H_
#define RGB_H_


struct RGB {
  RGB() : data_{0,0,0} {}
  RGB(uint8_t r, uint8_t g, uint8_t b) : data_{r, g, b} {}
  uint8_t data_[3];
  int sum() const { return (int)data_[0] + data_[1] + data_[2]; }
};

inline RGB operator-(const RGB& a, const RGB& b) {
  return RGB(
      a.data_[0] - b.data_[0],
      a.data_[1] - b.data_[1],
      a.data_[2] - b.data_[2]);
}
inline RGB operator+(const RGB& a, const RGB& b) {
  return RGB(
      a.data_[0] + b.data_[0],
      a.data_[1] + b.data_[1],
      a.data_[2] + b.data_[2]);
}
inline RGB operator*(const RGB& a, float b) {
  return RGB(
      a.data_[0] * b + 0.5,
      a.data_[1] * b + 0.5,
      a.data_[2] * b + 0.5);
}
inline RGB operator*(float a, const RGB& b) { return b * a; }

template<>
inline int abs_diff<RGB>(const RGB& a, const RGB& b) {
 return
     abs((int)a.data_[0] - (int)b.data_[0]) +
     abs((int)a.data_[1] - (int)b.data_[1]) +
     abs((int)a.data_[2] - (int)b.data_[2]);
}

template<>
inline int delta<RGB>(const RGB& a, const RGB& b) {
 return
     (((int)a.data_[0] - (int)b.data_[0]) +
     ((int)a.data_[1] - (int)b.data_[1]) +
     ((int)a.data_[2] - (int)b.data_[2]) + 2)/3;
}

template<>
inline RGB average(const RGB& a, const RGB& b, const RGB& c, const RGB& d) {
  RGB r = RGB(
      ((int)a.data_[0] + b.data_[0] + c.data_[0] + d.data_[0] + 3)/4,
      ((int)a.data_[1] + b.data_[1] + c.data_[1] + d.data_[1] + 3)/4,
      ((int)a.data_[2] + b.data_[2] + c.data_[2] + d.data_[2] + 3)/4);
  return r;
}

#endif
