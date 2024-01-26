#include "speed_data.h"

#include <algorithm>

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : speed_vector_(std::move(speed_points)) {}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
    if (!speed_vector_.empty()) {
        if (speed_vector_.back().t > time) return;
    }
    speed_vector_.push_back(SpeedPoint{s, time, v, a, da});
}
const std::vector<SpeedPoint> &SpeedData::SpeedVector() const {
    return speed_vector_;
}

void SpeedData::SetSpeedVector(std::vector<SpeedPoint> speed_points) {
    speed_vector_ = std::move(speed_points);
}

bool SpeedData::EvaluateByTime(const double t,
                               SpeedPoint *const speed_point) const {
    if (speed_vector_.size() < 2) {
        return false;
    }
    if (!(speed_vector_.front().t < t + 1.0e-6 &&
          t - 1.0e-6 < speed_vector_.back().t)) {
        return false;
    }
    auto comp = [](const SpeedPoint &sp, const double t) { return sp.t < t; };
    auto it_lower =
        std::lower_bound(speed_vector_.begin(), speed_vector_.end(), t, comp);

    if (it_lower == speed_vector_.end()) {
        *speed_point = speed_vector_.back();
    } else if (it_lower == speed_vector_.begin()) {
        *speed_point = speed_vector_.front();
    } else {
        const auto &p0 = *(it_lower - 1);
        const auto &p1 = *it_lower;
        double t0 = p0.t;
        double t1 = p1.t;

        double s = lerp(p0.s, t0, p1.s, t1, t);
        double v = lerp(p0.velocity, t0, p1.velocity, t1, t);
        double a = lerp(p0.acc, t0, p1.acc, t1, t);
        double j = lerp(p0.dot_acc, t0, p1.dot_acc, t1, t);

        *speed_point = SpeedPoint{s, t, v, a, j};
    }
    return true;
}
bool SpeedData::EvaluateTimeByS(const double s, double *t) const {
    if (s < speed_vector_.front().s || s > speed_vector_.back().s) {
        return false;
    }
    auto comp = [](const SpeedPoint &sp, const double s) { return sp.s < s; };
    auto it_lower =
        std::lower_bound(speed_vector_.begin(), speed_vector_.end(), s, comp);
    if (it_lower == speed_vector_.end()) {
        *t = speed_vector_.back().t;
    } else if (it_lower == speed_vector_.begin()) {
        *t = speed_vector_.front().t;
    } else {
        *t = it_lower->t;
    }
    return true;
}
double SpeedData::TotalTime() const {
    if (speed_vector_.empty()) {
        return 0.0;
    }
    return speed_vector_.back().t - speed_vector_.front().t;
}
void SpeedData::Clear() {
    speed_vector_.clear();
}
}  // namespace planning_lib
}  // namespace jarvis