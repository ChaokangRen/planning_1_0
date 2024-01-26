#pragma once

#include "common.h"
#include "vec2d.h"

namespace jarvis {
namespace planning_lib {

class LineSegment2d {
public:
    LineSegment2d();
    LineSegment2d(const Vec2d &start, const Vec2d &end);

    const Vec2d &Start() const {
        return start_;
    }

    const Vec2d &End() const {
        return end_;
    }

    const Vec2d &UnitDirection() const {
        return unit_direction_;
    }

    const double &Length() const {
        return length_;
    }

    Vec2d Center() const;

    double Heading() const {
        return heading_;
    }

    double CosHeading() const {
        return unit_direction_.x();
    }

    double SinHeading() const {
        return unit_direction_.y();
    }

    double CrossPord(const Vec2d &pos1, const Vec2d &pos2,
                     const Vec2d &pos3) const;

    double DistanceTo(const Vec2d &point) const;

    bool IsPointIn(const Vec2d &point) const;

    bool HasIntersect(const LineSegment2d &other_linesegment) const;

    bool HasIntersectWithPoint(const LineSegment2d &other_linesegment,
                               Vec2d *point) const;

    double ProjectOntoUnit(const Vec2d &point) const;

private:
    bool IsWithin(double val, double left_val, double right_val) const;

private:
    Vec2d start_;
    Vec2d end_;
    Vec2d unit_direction_;
    double heading_ = 0.0;
    double length_ = 0.0;
};
}  // namespace planning_lib
}  // namespace jarvis
