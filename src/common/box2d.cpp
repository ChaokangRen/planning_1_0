#include "box2d.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

#include "polygon2d.h"
namespace jarvis {
namespace planning_lib {
double BoxCrossPord(const Vec2d &pos1, const Vec2d &pos2, const Vec2d &pos3) {
    Vec2d vec1(pos2 - pos1);
    Vec2d vec2(pos3 - pos1);
    return vec1.CrossProd(vec2);
}

double PtSegDistance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y,
                     double length) {
    const double x0 = query_x - start_x;
    const double y0 = query_y - start_y;
    const double dx = end_x - start_x;
    const double dy = end_y - start_y;
    const double proj = x0 * dx + y0 * dy;
    if (proj <= 0.0) {
        return hypot(x0, y0);
    }
    if (proj >= length * length) {
        return hypot(x0 - dx, y0 - dy);
    }
    return std::abs(x0 * dy - y0 * dx) / length;
}
Box2d::Box2d(const Vec2d &center, const double heading, const double length,
             const double width)
    : center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {
    InitCorners();
}
Box2d::Box2d(const LineSegment2d &axis, const double width)
    : center_(axis.Center()),
      length_(axis.Length()),
      width_(width),
      half_length_(axis.Length() / 2.0),
      half_width_(width / 2.0),
      heading_(axis.Heading()),
      cos_heading_(axis.CosHeading()),
      sin_heading_(axis.SinHeading()) {
    if (length_ < kMathEpsilon || width_ < kMathEpsilon) {
        std::cout << "box2d length or width maybe wrong" << std::endl;
    }
    InitCorners();
}
Box2d Box2d::CreateAABox(const Vec2d &one_corner,
                         const Vec2d &opposite_corner) {
    const double x1 = std::min(one_corner.x(), opposite_corner.x());
    const double x2 = std::max(one_corner.x(), opposite_corner.x());
    const double y1 = std::min(one_corner.y(), opposite_corner.y());
    const double y2 = std::max(one_corner.y(), opposite_corner.y());
    return Box2d({(x1 + x2) / 2.0, (y1 + y2) / 2.0}, 0.0, x2 - x1, y2 - y1);
}
void Box2d::InitCorners() {
    const double dx1 = cos_heading_ * half_length_;
    const double dy1 = sin_heading_ * half_length_;
    const double dx2 = sin_heading_ * half_width_;
    const double dy2 = -cos_heading_ * half_width_;
    corners_.clear();
    corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
    corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
    corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
    corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

    for (Vec2d &corner : corners_) {
        max_x_ = std::fmax(corner.x(), max_x_);
        min_x_ = std::fmin(corner.x(), min_x_);
        max_y_ = std::fmax(corner.y(), max_y_);
        min_y_ = std::fmin(corner.y(), min_y_);
    }
}
bool Box2d::IsPointIn(const Vec2d &point) const {
    const double x0 = point.x() - center_.x();
    const double y0 = point.y() - center_.y();
    const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
    const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
    return dx <= half_length_ + kMathEpsilon &&
           dy <= half_width_ + kMathEpsilon;
}
bool Box2d::IsPointOnBoundary(const Vec2d &point) const {
    const double x0 = point.x() - center_.x();
    const double y0 = point.y() - center_.y();
    const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
    const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
    return (std::abs(dx - half_length_) <= kMathEpsilon &&
            dy <= half_width_ + kMathEpsilon) ||
           (std::abs(dy - half_width_) <= kMathEpsilon &&
            dx <= half_length_ + kMathEpsilon);
}
double Box2d::DistanceTo(const Vec2d &point) const {
    const double x0 = point.x() - center_.x();
    const double y0 = point.y() - center_.y();
    const double dx =
        std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
    const double dy =
        std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
    if (dx <= 0.0) {
        return std::max(0.0, dy);
    }
    if (dy <= 0.0) {
        return dx;
    }
    return hypot(dx, dy);
}

double Box2d::DistanceTo(const LineSegment2d &line_segment) const {
    if (line_segment.Length() <= kMathEpsilon) {
        return DistanceTo(line_segment.Start());
    }
    const double ref_x1 = line_segment.Start().x() - center_.x();
    const double ref_y1 = line_segment.Start().y() - center_.y();
    double x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
    double y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
    double box_x = half_length_;
    double box_y = half_width_;
    int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
    int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
    if (gx1 == 0 && gy1 == 0) {
        return 0.0;
    }
    const double ref_x2 = line_segment.End().x() - center_.x();
    const double ref_y2 = line_segment.End().y() - center_.y();
    double x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
    double y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
    int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
    int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
    if (gx2 == 0 && gy2 == 0) {
        return 0.0;
    }
    if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
        x1 = -x1;
        gx1 = -gx1;
        x2 = -x2;
        gx2 = -gx2;
    }
    if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
        y1 = -y1;
        gy1 = -gy1;
        y2 = -y2;
        gy2 = -gy2;
    }
    if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
        std::swap(x1, y1);
        std::swap(gx1, gy1);
        std::swap(x2, y2);
        std::swap(gx2, gy2);
        std::swap(box_x, box_y);
    }
    if (gx1 == 1 && gy1 == 1) {
        switch (gx2 * 3 + gy2) {
            case 4:
                return PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                     line_segment.Length());
            case 3:
                return (x1 > x2) ? (x2 - box_x)
                                 : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                                 line_segment.Length());
            case 2:
                return (x1 > x2) ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                                 line_segment.Length())
                                 : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                                 line_segment.Length());
            case -1:
                return BoxCrossPord({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                           ? 0.0
                           : PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                           line_segment.Length());
            case -4:
                return BoxCrossPord({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                           ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                           line_segment.Length())
                           : (BoxCrossPord({x1, y1}, {x2, y2},
                                           {-box_x, box_y}) <= 0.0
                                  ? 0.0
                                  : PtSegDistance(-box_x, box_y, x1, y1, x2, y2,
                                                  line_segment.Length()));
        }
    } else {
        switch (gx2 * 3 + gy2) {
            case 4:
                return (x1 < x2) ? (x1 - box_x)
                                 : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                                 line_segment.Length());
            case 3:
                return std::min(x1, x2) - box_x;
            case 1:
            case -2:
                return BoxCrossPord({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                           ? 0.0
                           : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                           line_segment.Length());
            case -3:
                return 0.0;
        }
    }
    return 0.0;
}
double Box2d::DistanceTo(const Box2d &box) const {
    return Polygon2d(box).DistanceTo(*this);
}
bool Box2d::HasOverlap(const LineSegment2d &line_segment) const {
    if (line_segment.Length() <= kMathEpsilon) {
        return IsPointIn(line_segment.Start());
    }
    if (std::fmax(line_segment.Start().x(), line_segment.End().x()) < min_x() ||
        std::fmin(line_segment.Start().x(), line_segment.End().x()) > max_x() ||
        std::fmax(line_segment.Start().y(), line_segment.End().y()) < min_x() ||
        std::fmin(line_segment.Start().y(), line_segment.End().y()) > max_x()) {
        return false;
    }
    return DistanceTo(line_segment) <= kMathEpsilon;
}
bool Box2d::HasOverlap(const Box2d &box) const {
    if (box.max_x() < min_x() || box.min_x() > max_x() ||
        box.max_y() < min_y() || box.min_y() > max_y()) {
        return false;
    }

    const double shift_x = box.CenterX() - center_.x();
    const double shift_y = box.CenterY() - center_.y();

    const double dx1 = cos_heading_ * half_length_;
    const double dy1 = sin_heading_ * half_length_;
    const double dx2 = sin_heading_ * half_width_;
    const double dy2 = -cos_heading_ * half_width_;
    const double dx3 = box.CosHeading() * box.HalfLength();
    const double dy3 = box.SinHeading() * box.HalfLength();
    const double dx4 = box.SinHeading() * box.HalfWidth();
    const double dy4 = -box.CosHeading() * box.HalfWidth();

    return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
               std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                   std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                   half_length_ &&
           std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
               std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                   std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                   half_width_ &&
           std::abs(shift_x * box.CosHeading() + shift_y * box.SinHeading()) <=
               std::abs(dx1 * box.CosHeading() + dy1 * box.SinHeading()) +
                   std::abs(dx2 * box.CosHeading() + dy2 * box.SinHeading()) +
                   box.HalfLength() &&
           std::abs(shift_x * box.SinHeading() - shift_y * box.CosHeading()) <=
               std::abs(dx1 * box.SinHeading() - dy1 * box.CosHeading()) +
                   std::abs(dx2 * box.SinHeading() - dy2 * box.CosHeading()) +
                   box.HalfWidth();
}

double Box2d::DistToBoxByRoundCheck(const Box2d &box) const {
    double dist_sqr =
        (center_.x() - box.center_.x()) * (center_.x() - box.center_.x()) +
        (center_.y() - box.center_.y()) * (center_.y() - box.center_.y());
    double r1 = std::max(length_, width_) + 1.0;
    double r2 = std::max(box.Length(), box.Width()) + 1.0;

    return std::sqrt(dist_sqr) - r1 - r2;
}
}  // namespace planning_lib
}  // namespace jarvis