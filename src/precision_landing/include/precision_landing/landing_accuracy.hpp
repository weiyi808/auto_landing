#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace precision_landing {

struct LandingAccuracySample {
  double stamp_s = 0.0;
  double forward_m = 0.0;
  double right_m = 0.0;
  double down_m = 0.0;
};

struct LandingAccuracyResult {
  bool valid = false;
  LandingAccuracySample sample;
  double horizontal_error_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t in_window_count = 0U;
};

inline LandingAccuracyResult selectFinalAccuracy(
    const std::vector<LandingAccuracySample>& samples, double touchdown_s,
    double window_s) {
  LandingAccuracyResult result;
  if (!std::isfinite(touchdown_s) || !std::isfinite(window_s) ||
      window_s < 0.0) {
    return result;
  }

  const double window_start_s = touchdown_s - window_s;
  for (const LandingAccuracySample& sample : samples) {
    const bool finite = std::isfinite(sample.stamp_s) &&
                        std::isfinite(sample.forward_m) &&
                        std::isfinite(sample.right_m) &&
                        std::isfinite(sample.down_m);
    if (!finite || sample.stamp_s < window_start_s ||
        sample.stamp_s > touchdown_s) {
      continue;
    }

    ++result.in_window_count;
    if (!result.valid || sample.stamp_s > result.sample.stamp_s) {
      result.valid = true;
      result.sample = sample;
      result.horizontal_error_m =
          std::hypot(sample.forward_m, sample.right_m);
    }
  }
  return result;
}

}  // namespace precision_landing
