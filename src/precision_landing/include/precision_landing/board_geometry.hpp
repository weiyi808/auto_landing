#pragma once

#include <array>

namespace precision_landing {

struct BoardPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

using BoardCorners = std::array<BoardPoint, 4>;

inline bool boardTagDefinition(int id, double* center_x, double* center_y,
                               double* black_size) {
  switch (id) {
    case 10: *center_x = -0.101;  *center_y = -0.0575; *black_size = 0.068;  return true;
    case 11: *center_x =  0.101;  *center_y = -0.0575; *black_size = 0.068;  return true;
    case 12: *center_x = -0.101;  *center_y =  0.0575; *black_size = 0.068;  return true;
    case 13: *center_x =  0.101;  *center_y =  0.0575; *black_size = 0.068;  return true;
    case 20: *center_x = -0.0365; *center_y = -0.039;  *black_size = 0.0288; return true;
    case 21: *center_x =  0.0365; *center_y = -0.039;  *black_size = 0.0288; return true;
    case 22: *center_x = -0.0365; *center_y =  0.039;  *black_size = 0.0288; return true;
    case 23: *center_x =  0.0365; *center_y =  0.039;  *black_size = 0.0288; return true;
    case 0:  *center_x =  0.0;    *center_y =  0.0;    *black_size = 0.0144; return true;
    default: return false;
  }
}

inline double blackMarkerSizeMeters(int id, double scale) {
  double center_x = 0.0;
  double center_y = 0.0;
  double black_size = 0.0;
  if (scale <= 0.0 ||
      !boardTagDefinition(id, &center_x, &center_y, &black_size)) {
    return 0.0;
  }
  return black_size * scale;
}

inline bool boardObjectCorners(int id, double scale, BoardCorners* points) {
  if (points == nullptr || scale <= 0.0) return false;
  double center_x = 0.0;
  double center_y = 0.0;
  double black_size = 0.0;
  if (!boardTagDefinition(id, &center_x, &center_y, &black_size)) return false;

  center_x *= scale;
  center_y *= scale;
  const double half = black_size * scale * 0.5;
  // The generated artwork is decoded 180 degrees from page orientation.
  // OpenCV therefore returns page BR, BL, TL, TR as canonical corners 0..3.
  *points = {{{center_x + half, center_y + half, 0.0},
              {center_x - half, center_y + half, 0.0},
              {center_x - half, center_y - half, 0.0},
              {center_x + half, center_y - half, 0.0}}};
  return true;
}

}  // namespace precision_landing
