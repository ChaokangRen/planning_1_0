#include "st_point.h"

namespace jarvis {
namespace planning_lib {
STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const Vec2d &vec2d_point) : Vec2d(vec2d_point) {}

double STPoint::s() const {
    return y();
}

double STPoint::t() const {
    return x();
}

void STPoint::set_s(const double s) {
    set_y(s);
}

void STPoint::set_t(const double t) {
    set_x(t);
}
}  // namespace planning_lib
}  // namespace jarvis