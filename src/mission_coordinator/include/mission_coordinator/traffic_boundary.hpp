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

private:
  std::vector<double> bounds_;
};
} // namespace mission_coordinator
