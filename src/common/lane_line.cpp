#include "lane_line.h"
namespace jarvis {
namespace planning_lib {

double LaneLine::GetLaneLineBoundary(const double s) const {
    if (lane_line_.empty()) {
        return 0.0;
    }
    if (s < 0.0) {
        return lane_line_[0].l;
    }
    int32_t index = lane_line_.size();
    for (int32_t i = 0; i < lane_line_.size(); ++i) {
        if (lane_line_[i].s <= s && lane_line_[i + 1].s > s) {
            index = i;
            break;
        }
    }
    if (index >= lane_line_.size() - 1) {
        return lane_line_.back().l;
    }
    double sample_distance = lane_line_[index + 1].s - lane_line_[index].s;

    const double ratio = (s - lane_line_[index].s) / sample_distance;

    return lane_line_[index].l * (1.0 - ratio) +
           lane_line_[index + 1].l * ratio;
}
}  // namespace planning_lib
}  // namespace jarvis