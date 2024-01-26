#pragma once

#include <limits>
#include <string>
#include <vector>

#include "common.h"
#include "line_segment2d.h"
namespace jarvis {
namespace planning_lib {
class Box2d {
public:
    Box2d() = default;
    Box2d(const Vec2d &center, const double heading, const double length,
          const double width);
    Box2d(const LineSegment2d &axis, const double width);
    void InitCorners();

    static Box2d CreateAABox(const Vec2d &one_corner,
                             const Vec2d &opposite_corner);
    const Vec2d &Center() const {
        return center_;
    }
    double CenterX() const {
        return center_.x();
    }
    double CenterY() const {
        return center_.y();
    }
    double Length() const {
        return length_;
    }
    double Width() const {
        return width_;
    }
    double HalfLength() const {
        return half_length_;
    }
    double HalfWidth() const {
        return half_width_;
    }
    double Heading() const {
        return heading_;
    }
    double CosHeading() const {
        return cos_heading_;
    }
    double SinHeading() const {
        return sin_heading_;
    }
    double Area() const {
        return length_ * width_;
    }
    double Diagonal() const {
        return std::hypot(length_, width_);
    }
    std::vector<Vec2d> GetAllCorners() const {
        return corners_;
    };
    bool IsPointIn(const Vec2d &point) const;
    bool IsPointOnBoundary(const Vec2d &point) const;
    double DistanceTo(const Vec2d &point) const;
    double DistanceTo(const LineSegment2d &line_segment) const;
    double DistanceTo(const Box2d &box) const;
    bool HasOverlap(const LineSegment2d &line_segment) const;
    bool HasOverlap(const Box2d &box) const;

    double DistToBoxByRoundCheck(const Box2d &box) const;

    double max_x() const {
        return max_x_;
    }
    double min_x() const {
        return min_x_;
    }
    double max_y() const {
        return max_y_;
    }
    double min_y() const {
        return min_y_;
    }

private:
    Vec2d center_;
    double length_ = 0.0;
    double width_ = 0.0;
    double half_length_ = 0.0;
    double half_width_ = 0.0;
    double heading_ = 0.0;
    double cos_heading_ = 1.0;
    double sin_heading_ = 0.0;

    std::vector<Vec2d> corners_;

    double max_x_ = std::numeric_limits<double>::min();
    double min_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::min();
    double min_y_ = std::numeric_limits<double>::max();
};
}  // namespace planning_lib
}  // namespace jarvis