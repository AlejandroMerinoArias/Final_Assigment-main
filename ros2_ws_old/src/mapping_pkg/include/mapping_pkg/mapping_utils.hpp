#pragma once

#include <cmath>

namespace mapping_pkg {

inline float clamp_range(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

inline bool is_finite(float value) {
  return std::isfinite(value);
}

}  // namespace mapping_pkg