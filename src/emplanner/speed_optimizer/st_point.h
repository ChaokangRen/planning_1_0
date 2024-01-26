#pragma once
#include <string>

#include "common.h"
#include "vec2d.h"

namespace jarvis {
namespace planning_lib {
class STPoint : public Vec2d {
public:
    STPoint() = default;
    STPoint(const double s, const double t);
    explicit STPoint(const Vec2d &Vec2d_point);

    double s() const;
    double t() const;
    void set_s(const double s);
    void set_t(const double t);
};
}  // namespace planning_lib
}  // namespace jarvis
