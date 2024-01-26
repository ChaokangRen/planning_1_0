#include "polygon2d.h"

#include <algorithm>
#include <cmath>

#include "common.h"
#include "line_segment2d.h"
namespace jarvis {
namespace planning_lib {
Polygon2d::Polygon2d(std::vector<Vec2d> points) : points_(std::move(points)) {
    BuildFromPoints();
}
Polygon2d::Polygon2d(const Box2d &box) {
    points_ = box.GetAllCorners();
    BuildFromPoints();
}
double Polygon2d::VecCrossPord(const Vec2d &pos1, const Vec2d &pos2,
                               const Vec2d &pos3) {
    Vec2d vec1(pos2 - pos1);
    Vec2d vec2(pos3 - pos1);
    return vec1.CrossProd(vec2);
}
double Polygon2d::DistanceTo(const Vec2d &point) const {
    if (IsPointInPolygon(point)) {
        return 0.0;
    }
    double distance = std::numeric_limits<double>::infinity();
    for (int32_t i = 0; i < points_num_; ++i) {
        distance = std::min(distance, line_segments_[i].DistanceTo(point));
    }
    return distance;
}
double Polygon2d::DistanceTo(const LineSegment2d &line_segment) const {
    if (line_segment.Length() < kMathEpsilon) {
        return DistanceTo(line_segment.Start());
    }
    // if line_segment has intersect with line_segments of polygon
    if (std::any_of(line_segments_.begin(), line_segments_.end(),
                    [&](const LineSegment2d &segments_of_poly) {
                        return segments_of_poly.HasIntersect(line_segment);
                    })) {
        return 0.0;
    }

    double distance = std::min(DistanceTo(line_segment.Start()),
                               DistanceTo(line_segment.End()));
    for (int i = 0; i < points_num_; ++i) {
        distance = std::min(distance, line_segment.DistanceTo(points_[i]));
    }
    return distance;
}
double Polygon2d::DistanceTo(const Polygon2d &polygon) const {
    if (IsPointInPolygon(polygon.points()[0])) {
        return 0.0;
    }
    if (polygon.IsPointInPolygon(points_[0])) {
        return 0.0;
    }
    double distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < points_num_; ++i) {
        distance = std::min(distance, polygon.DistanceTo(line_segments_[i]));
    }
    return distance;
}
double Polygon2d::DistanceTo(const Box2d &box) const {
    return DistanceTo(Polygon2d(box));
}
// Ray method to find whether points are in polygons:see
// https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
bool Polygon2d::IsPointInPolygon(const Vec2d &point) const {
    if (IsPointOnSegments(point)) {
        return true;
    }
    bool is_point_in_polygon = false;
    for (int i = 0, j = points_num_ - 1; i < points_num_; j = i++) {
        if (((points_[i].y() > point.y()) != (points_[j].y() > point.y())) &&
            (point.x() < (points_[j].x() - points_[i].x()) *
                                 (point.y() - points_[i].y()) /
                                 (points_[j].y() - points_[i].y()) +
                             points_[i].x()))
            is_point_in_polygon = !is_point_in_polygon;
    }
    return is_point_in_polygon;
}
bool Polygon2d::IsPointOnSegments(const Vec2d &point) const {
    return std::any_of(line_segments_.begin(), line_segments_.end(),
                       [&](const LineSegment2d &segments_of_poly) {
                           return segments_of_poly.IsPointIn(point);
                       });
}
void Polygon2d::BuildFromPoints(void) {
    points_num_ = static_cast<int>(points_.size());

    // Calculate the polygon area by cross product
    area_ = 0.0;
    for (int32_t i = 1; i < points_num_; ++i) {
        area_ += VecCrossPord(points_[0], points_[i - 1], points_[i]);
    }
    if (area_ < 0) {
        area_ = -area_;
        std::reverse(points_.begin(), points_.end());
    }
    area_ = area_ / 2;

    // Initialize the linesegments
    for (int32_t i = 0; i < points_num_; ++i) {
        line_segments_.emplace_back(points_[i], points_[Next(i)]);
    }

    // cheack convexity
    is_convex_ = true;
    for (int32_t i = 0; i < points_num_; ++i) {
        if (VecCrossPord(points_[Prev(i)], points_[i], points_[Next(i)]) <=
            kMathEpsilon) {
            is_convex_ = false;
            break;
        }
    }

    min_x_ = points_[0].x();
    min_y_ = points_[0].y();
    max_x_ = points_[0].x();
    max_y_ = points_[0].y();
    for (const Vec2d &point : points_) {
        min_x_ = std::min(min_x_, point.x());
        min_y_ = std::min(min_y_, point.y());
        max_x_ = std::max(max_x_, point.x());
        max_y_ = std::max(max_y_, point.y());
    }
}

bool Polygon2d::HasOverlap(const LineSegment2d &line_segments) const {
    if ((line_segments.Start().x() < min_x_ &&
         line_segments.End().x() < min_x_) ||
        (line_segments.Start().x() > max_x_ &&
         line_segments.End().x() > max_x_) ||
        (line_segments.Start().y() > max_y_ &&
         line_segments.End().y() > max_y_) ||
        (line_segments.Start().y() > max_y_ &&
         line_segments.End().x() > max_y_)) {
        return false;
    }
    Vec2d first;
    Vec2d second;
    return GetOverlap(line_segments, &first, &second);
}
bool Polygon2d::GetOverlap(const LineSegment2d &line_segment,
                           Vec2d *const first, Vec2d *const last) const {
    if (line_segment.Length() < kMathEpsilon) {
        if (!IsPointInPolygon(line_segment.Start())) {
            return false;
        }
        *first = line_segment.Start();
        *last = line_segment.End();
    }

    double min_proj = line_segment.Length();
    double max_proj = 0;
    if (IsPointInPolygon(line_segment.Start())) {
        *first = line_segment.Start();
        min_proj = 0.0;
    }
    if (IsPointInPolygon(line_segment.End())) {
        *last = line_segment.End();
        max_proj = line_segment.Length();
    }
    for (const LineSegment2d &poly_seg : line_segments_) {
        Vec2d pt(0, 0);
        if (poly_seg.HasIntersectWithPoint(line_segment, &pt)) {
            const double proj = line_segment.ProjectOntoUnit(pt);
            if (proj < min_proj) {
                min_proj = proj;
                *first = pt;
            }
            if (proj > max_proj) {
                max_proj = proj;
                *last = pt;
            }
        }
    }
    return min_proj <= max_proj + kMathEpsilon;
}
}  // namespace planning_lib
}  // namespace jarvis