// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>
namespace mission_coordinator {
// Axis-aligned exclusive polygon in the robot's world-aligned map.
class TrafficBoundary {
public:
  explicit TrafficBoundary(std::vector<double> bounds) : bounds_(std::move(bounds)) {
    if (bounds_.size() != 4 ||
        !std::all_of(bounds_.begin(), bounds_.end(),
                     [](double v) { return std::isfinite(v); }) ||
        bounds_[0] >= bounds_[2] || bounds_[1] >= bounds_[3])
      throw std::invalid_argument(
          "traffic bounds require finite [xmin, ymin, xmax, ymax]");
  }
  bool outside(double x, double y, double margin) const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(margin) &&
           margin >= 0 &&
           (x + margin < bounds_[0] || y + margin < bounds_[1] ||
            x - margin > bounds_[2] || y - margin > bounds_[3]);
  }
  bool segment_outside(double x1, double y1, double x2, double y2,
                       double margin) const {
    if (!outside(x1, y1, margin) || !outside(x2, y2, margin))
      return false;
    // Closed slab intersection: touching the protected footprint fails closed.
    double entry = 0, exit = 1;
    for (unsigned axis = 0; axis < 2; ++axis) {
      const double start = axis == 0 ? x1 : y1;
      const double delta = axis == 0 ? x2 - x1 : y2 - y1;
      const double lower = bounds_[axis] - margin;
      const double upper = bounds_[axis + 2] + margin;
      if (delta == 0) {
        if (start < lower || start > upper)
          return true;
      } else {
        const double a = (lower - start) / delta;
        const double b = (upper - start) / delta;
        entry = std::max(entry, std::min(a, b));
        exit = std::min(exit, std::max(a, b));
        if (entry > exit)
          return true;
      }
    }
    return false;
  }

private:
  std::vector<double> bounds_;
};
} // namespace mission_coordinator
