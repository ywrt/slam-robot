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
inline bool operator < (const Pos& a, const Pos& b) {
  if (a.y != b.y)
    return a.y < b.y;
  return a.x < b.x;
}
inline bool operator > (const Pos& a, const Pos& b) {
  if (a.y != b.y)
    return a.y > b.y;
  return a.x > b.x;
}

// A coordinate space.
struct Space {
  int width;
  int height;
  int stride;
  Space(int w, int h) : width(w), height(h), stride(w) {}
  Space(int w, int h, int s) : width(w), height(h), stride(s) {}

  inline int size() const { return stride * height; }

  inline Pos toPos(const FPos& fpos) const {
    return Pos(fpos.x * width, fpos.y * height);
  }

  inline FPos toFPos(const Pos& p) const {
    float x = (float(p.x)+0.01f) / width;
    float y = (float(p.y)+0.01f) / height;
    return FPos(x, y);
  }

  inline int index(const Pos& p) const {
    return p.x + p.y * stride;
  }
  inline int index(int x, int y) const {
    return x + y * stride;
  }

  Pos clip(const Pos& pos) const {
    Pos r = pos;
    if (r.x < 0) r.x = 0;
    if (r.y < 0) r.y = 0;
    if ((width - r.x) <= 0) r.x = width - 1;
    if ((height - r.y) <= 0) r.y = height - 1;
    return r;
  }

  Pos clip(const Pos& pos, int margin) const {
    Pos r = pos;
    if (r.x < margin) r.x = margin;
    if (r.y < margin) r.y = margin;
    if ((width - r.x) <= margin) r.x = width - margin - 1;
    if ((height - r.y) <= margin) r.y = height - margin - 1;
    return r;
  }

  inline bool contains(const Pos& pos, int margin) const {
    Pos r = pos;
    if (r.x < margin) return false;
    if (r.y < margin) return false;
    if ((width - r.x) <= margin) return false;
    if ((height - r.y) <= margin) return false;
    return true;
  }
};

inline  bool operator == (const Space& a, const Space& b) {
  return a.width == b.width && a.height == b.height;
}
inline  bool operator != (const Space& a, const Space& b) {
  return !(a == b);
}

#endif
