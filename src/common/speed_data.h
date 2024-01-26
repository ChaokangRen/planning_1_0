#pragma once

#include "common.h"

namespace jarvis {
namespace planning_lib {
class SpeedData {
public:
    SpeedData() = default;
    explicit SpeedData(std::vector<SpeedPoint> speed_points);

    virtual ~SpeedData() = default;

    const std::vector<SpeedPoint> &SpeedVector() const;

    void SetSpeedVector(std::vector<SpeedPoint> speed_points);

    void AppendSpeedPoint(const double s, const double time, const double v,
                          const double a, const double da);
    bool EvaluateByTime(const double time, SpeedPoint *const speed_point) const;

    bool EvaluateTimeByS(const double s, double *t) const;

    double TotalTime() const;

    bool Empty() const {
        return speed_vector_.empty();
    }

    void Clear();

    std::vector<SpeedPoint> GetSpeedVector() {
        return speed_vector_;
    }

private:
    std::vector<SpeedPoint> speed_vector_;
};
}  // namespace planning_lib
}  // namespace jarvis