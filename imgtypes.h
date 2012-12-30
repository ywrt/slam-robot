#ifndef IMGTYPES_H
#define IMGTYPES_H

struct FPos {
  float x;
  float y;
  inline FPos(float a, float b) : x(a), y(b) {}
  inline FPos(double a, double b) : x(a), y(b) {}
  inline FPos(int a, int b) : x(float(a)), y(float(b)) {}
  inline FPos() : x(0), y(0) { }
  bool isInvalid() { return x <= -1 && y <= -1; }
  static FPos invalid() { return FPos(-2, -2); }
};
inline FPos operator + (const FPos& pos, const FPos& a) {
  FPos r(pos);
  r.x += a.x;
  r.y += a.y;
  return r;
}
inline FPos operator - (const FPos& pos, const FPos& a) {
  FPos r(pos);
  r.x = pos.x - a.x;
  r.y = pos.y - a.y;
  return r;
}

struct Pos {
  int x;
  int y;
  inline Pos(int a, int b) : x(a), y(b) {}
  inline Pos() : x(0), y(0) { }
  static Pos invalid() { return Pos(-1, -1); }
};

inline Pos operator + (const Pos& pos, const int& a) {
  Pos r(pos);
  r.x += a;
  r.y += a;
  return r;
}
inline Pos operator + (const Pos& pos, const Pos& a) {
  Pos r(pos);
  r.x += a.x;
  r.y += a.y;
  return r;
}
inline Pos operator - (const Pos& pos, const int& a) {
  return pos + (-a);
}
inline Pos operator - (const Pos& pos, const Pos& a) {
  Pos r(pos);
  r.x -= a.x;
  r.y -= a.y;
  return r;
}
inline Pos operator * (const Pos& pos, int a) {
  Pos r(pos);
  r.x *= a;
  r.y *= a;
  return r;
}
inline  bool operator == (const Pos& a, const Pos& b) {
  return a.x == b.x && a.y == b.y;
}
inline  bool operator != (const Pos& a, const Pos& b) {
  return !(a == b);
}


#endif