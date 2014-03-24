
#include "imgtypes.h"

std::ostream& operator<<(std::ostream& stream, const Pos& pos) {
  return stream << "[ " << pos.x << ", " << pos.y << "]";
}
std::ostream& operator<<(std::ostream& stream, const FPos& pos) {
  return stream << "[ " << pos.x << ", " << pos.y << "]";
}
