#include "line_segment2d.h"

#include <cmath>
#include <iostream>

#include "common.h"
namespace jarvis {
namespace planning_lib {
LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
    const double delta_x = end_.x() - start_.x();
    const double delta_y = end_.y() - start_.y();
    length_ = hypot(delta_x, delta_y);

    unit_direction_ =
        (length_ <= kMathEpsilon ? Vec2d{0, 0}
                                 : Vec2d{delta_x / length_, delta_y / length_});
    heading_ = std::atan2(unit_direction_.y(), unit_direction_.x());
}
double LineSegment2d::DistanceTo(const Vec2d &point) const {
    if (length_ <= kMathEpsilon) {
        return start_.DistanceTo(point);
    }
    const double delta_x = point.x() - start_.x();
    const double delta_y = point.y() - start_.y();
    const double proj_to_line_length =
        delta_x * unit_direction_.x() + delta_y * unit_direction_.y();
    if (proj_to_line_length <= 0.0) {
        return std::hypot(delta_x, delta_y);
    }
    if (proj_to_line_length > length_) {
        return end_.DistanceTo(point);
    }
    return std::abs(delta_x * unit_direction_.y() -
                    delta_y * unit_direction_.x());
}
Vec2d LineSegment2d::Center() const {
    return Vec2d((start_.x() + end_.x()) / 2, (start_.y() + end_.y()) / 2);
}
double LineSegment2d::CrossPord(const Vec2d &pos1, const Vec2d &pos2,
                                const Vec2d &pos3) const {
    Vec2d vec1(pos2.x() - pos1.x(), pos2.y() - pos1.y());
    Vec2d vec2(pos3.x() - pos1.x(), pos3.y() - pos1.y());
    return vec1.CrossProd(vec2);
}
bool LineSegment2d::IsWithin(double val, double left_val,
                             double right_val) const {
    if (left_val > right_val) {
        std::swap(left_val, right_val);
    }
    return val >= left_val - kMathEpsilon && val <= right_val + kMathEpsilon;
}
bool LineSegment2d::IsPointIn(const Vec2d &point) const {
    if (length_ <= kMathEpsilon) {
        return std::abs(point.x() - start_.x()) <= kMathEpsilon &&
               std::abs(point.y() - start_.y()) <= kMathEpsilon;
    }
    // If the area formed by a point and a line segment is large, the point is
    // not on the line segment
    double prod = CrossPord(point, start_, end_);
    if (std::abs(prod) > kMathEpsilon) {
        return false;
    }
    return IsWithin(point.x(), start_.x(), end_.x()) &&
           IsWithin(point.y(), start_.y(), end_.y());
}

bool LineSegment2d::HasIntersect(const LineSegment2d &other_linesegment) const {
    Vec2d point;
    return HasIntersectWithPoint(other_linesegment, &point);
}

bool LineSegment2d::HasIntersectWithPoint(
    const LineSegment2d &other_linesegment, Vec2d *point) const {
    if (IsPointIn(other_linesegment.Start())) {
        *point = other_linesegment.Start();
        return true;
    }
    if (IsPointIn(other_linesegment.End())) {
        *point = other_linesegment.End();
        return true;
    }
    if (other_linesegment.IsPointIn(start_)) {
        *point = start_;
        return true;
    }
    if (other_linesegment.IsPointIn(end_)) {
        *point = end_;
        return true;
    }
    if (length_ <= kMathEpsilon || other_linesegment.Length() <= kMathEpsilon) {
        return false;
    }
    double crosspord_1 = CrossPord(start_, end_, other_linesegment.Start());
    double crosspord_2 = CrossPord(start_, end_, other_linesegment.End());
    if (crosspord_1 * crosspord_2 >= -kMathEpsilon) {
        return false;
    }
    double crosspord_3 =
        CrossPord(other_linesegment.Start(), other_linesegment.End(), start_);
    double crosspord_4 =
        CrossPord(other_linesegment.Start(), other_linesegment.End(), end_);
    if (crosspord_3 * crosspord_4 >= -kMathEpsilon) {
        return false;
    }
    double ratio = crosspord_4 / (crosspord_4 - crosspord_3);
    *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                   start_.y() * ratio + end_.y() * (1.0 - ratio));
    return true;
}
double LineSegment2d::ProjectOntoUnit(const Vec2d &point) const {
    Vec2d point_vec(point.x() - start_.x(), point.y() - start_.y());
    return unit_direction_.InnerProd(point_vec);
}
}  // namespace planning_lib
}  // namespace jarvis