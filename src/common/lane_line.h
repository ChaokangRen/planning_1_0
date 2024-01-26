#pragma once

#include <vector>

#include "common.h"

namespace jarvis {
namespace planning_lib {
class LaneLine {
public:
    LaneLine() = default;

    LaneLine(std::vector<SLPoint> lane_line) : lane_line_(lane_line) {}

    double GetLaneLineBoundary(const double s) const;

    double GetLaneLineBoundaryByIndex(double i) const {
        return lane_line_[i].l;
    }

    void emplace_back(const SLPoint &sl_point) {
        lane_line_.emplace_back(sl_point);
    }

    std::vector<SLPoint> GetLine() {
        return lane_line_;
    }

private:
    std::vector<SLPoint> lane_line_;
};
}  // namespace planning_lib
}  // namespace jarvis