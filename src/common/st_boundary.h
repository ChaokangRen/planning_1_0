#pragma once
#include <limits>
#include <string>
#include <vector>

#include "line_segment2d.h"
#include "polygon2d.h"
#include "st_point.h"

namespace jarvis {
namespace planning_lib {
class StBoundary : public Polygon2d {
public:
    StBoundary() = default;
    explicit StBoundary(
        const std::vector<std::pair<STPoint, STPoint>> &point_paris);
    explicit StBoundary(const Polygon2d *polygon) = delete;
    explicit StBoundary(std::vector<Vec2d> points) = delete;

    ~StBoundary() = default;

    bool IsEmpty() const {
        return lower_points_.empty();
    }
    void SetSpeed(const double speed) {
        obs_speed_ = speed;
    }
    double GetSpeed(void) {
        return obs_speed_;
    }
    void SetIsVirtual(const bool is_virtual) {
        is_virtual_ = is_virtual;
    }
    bool GetIsVitual(void) {
        return is_virtual_;
    }
    void SetIsFrontObs(const bool is_front_obs) {
        is_front_obs_ = is_front_obs;
    }
    bool IsFrontObs(void) {
        return is_front_obs_;
    }
    bool IsPointInBoundary(const STPoint &st_point) const;

    STPoint BottomLeftPoint() const;
    STPoint BottomRightPoint() const;

    StBoundary ExpandByS(const double s) const;
    StBoundary ExpandByT(const double t) const;

    //
    enum class BoundaryType {
        UNKNOWN,
        STOP,
        FOLLOW,
        YIELD,
        OVERTAKE,
        KEEP_CLEAR,
    };
    const std::string &id() const;
    void SetId(const std::string &id);

    bool GetUnblockSRange(const double curr_time, double *s_upper,
                          double *s_lower) const;
    bool GetBoundarySRange(const double curr_time, double *s_upper,
                           double *s_lower) const;
    BoundaryType boundary_type() const;
    void SetBoundaryType(const BoundaryType &boundary_type);
    double MinS() const;
    double MinT() const;
    double MaxS() const;
    double MaxT() const;
    std::vector<STPoint> upper_points() const;
    std::vector<STPoint> lower_points() const;

    static StBoundary GenerateStBoundary(
        const std::vector<STPoint> &lower_points,
        const std::vector<STPoint> &upper_points);
    StBoundary CutOffByT(const double t) const;

private:
    bool IsPointNear(const LineSegment2d &seg, const Vec2d &point,
                     const double max_dist);
    void RemoveRedundantPoints(
        std::vector<std::pair<STPoint, STPoint>> *point_pairs);
    bool GetIndexRange(const std::vector<STPoint> &points, const double t,
                       int32_t *left, int32_t *right) const;

private:
    BoundaryType boundary_type_ = BoundaryType::UNKNOWN;
    std::vector<STPoint> upper_points_;
    std::vector<STPoint> lower_points_;

    double area_ = 0.0;

    std::string id_;
    double s_up_limit_ = 100.0;

    double min_s_ = std::numeric_limits<double>::max();
    double max_s_ = std::numeric_limits<double>::lowest();
    double min_t_ = std::numeric_limits<double>::max();
    double max_t_ = std::numeric_limits<double>::lowest();

    double obs_speed_ = 0.0;
    bool is_virtual_ = false;
    bool is_front_obs_ = false;
};
}  // namespace planning_lib
}  // namespace jarvis