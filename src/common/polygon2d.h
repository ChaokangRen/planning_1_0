#pragma once

#include "box2d.h"
#include "common.h"
#include "line_segment2d.h"
namespace jarvis {
namespace planning_lib {
class Polygon2d {
public:
    Polygon2d() = default;
    Polygon2d(std::vector<Vec2d> points);
    explicit Polygon2d(const Box2d &box);
    double DistanceTo(const LineSegment2d &line_segment) const;
    double DistanceTo(const Polygon2d &polygon) const;
    double DistanceTo(const Box2d &box) const;
    double DistanceTo(const Vec2d &point) const;
    std::vector<Vec2d> points(void) const {
        return points_;
    };
    std::vector<LineSegment2d> line_segments(void) const {
        return line_segments_;
    }
    bool HasOverlap(const LineSegment2d &line_segments) const;
    bool GetOverlap(const LineSegment2d &line_segment, Vec2d *const first,
                    Vec2d *const last) const;
    double Area() {
        return area_;
    }
    double MinY() const {
        return min_y_;
    }
    double MinX() const {
        return min_x_;
    }
    double MaxX() const {
        return max_x_;
    }
    double MaxY() const {
        return max_y_;
    }

private:
    double VecCrossPord(const Vec2d &pos1, const Vec2d &pos2,
                        const Vec2d &pos3);
    bool IsPointInPolygon(const Vec2d &point) const;
    bool IsPointOnSegments(const Vec2d &point) const;

protected:
    void BuildFromPoints(void);
    std::vector<Vec2d> points_;

    std::vector<LineSegment2d> line_segments_;
    int32_t Next(int at) const {
        return at >= points_num_ - 1 ? 0 : at + 1;
    }
    int32_t Prev(int at) const {
        return at == 0 ? points_num_ - 1 : at - 1;
    }
    int32_t points_num_ = 0;
    bool is_convex_ = false;
    double area_ = 0;
    double min_x_ = 0;
    double max_x_ = 0;
    double min_y_ = 0;
    double max_y_ = 0;
};
}  // namespace planning_lib
}  // namespace jarvis
