#pragma once

#include <algorithm>
#include <memory>

#include "common.h"
#include "obstacle.h"
#include "reference_line.h"
#include "speed_data.h"

namespace jarvis {
namespace planning_lib {

enum class LcDirection { LaneKeep = 0, TurnLeft = 1, TurnRight = 2 };

class PathBoundsDecider {
public:
    PathBoundsDecider() = default;

    explicit PathBoundsDecider(const SpeedData &heuristic_speed_data)
        : heuristic_speed_data_(&heuristic_speed_data) {}

    void Process(
        const SpeedData &heuristic_speed_data,
        const std::vector<Obstacle> &obstacles, const DpPathData &dp_path,
        const ReferenceLine *refline,
        std::vector<std::pair<LaneLine, LaneLine>> &sl_navigable_areas);
    void CalculateQpBoundaryConstraint(const std::vector<Obstacle> &obstacles,
                                       const ReferenceLine *refline,
                                       const SLPoint &ref_point,
                                       double *left_boundary,
                                       double *right_boundary);
    void SetStraightBound(const std::vector<Obstacle> &obstacles,
                          const DpPathData &dp_path,
                          const ReferenceLine *refline,
                          LaneLine &left_navigable_area_,
                          LaneLine &right_navigable_area_);
    void SetLcBound(const std::vector<Obstacle> &obstacles,
                    const DpPathData &dp_path, const ReferenceLine *refline,
                    LaneLine &left_navigable_area_,
                    LaneLine &right_navigable_area);
    void SetBound(const std::vector<Obstacle> &obstacles,
                  const DpPathData &dp_path, const ReferenceLine *refline,
                  LaneLine &left_navigable_area_,
                  LaneLine &right_navigable_area_);

private:
    void CalculateStaticObsConstraint(const SLPoint &ref_point,
                                      const SLStaticBoundary &obstacle,
                                      double *left_boundary,
                                      double *right_boundary);
    void CalculateDynamicObsConstraint(
        const std::vector<Obstacle> &obstacles, const ReferenceLine *refline,
        const SLPoint &ref_point, double *left_boundary, double *right_boundary,
        double left_lane_boundary, double right_lane_boundary);

    const SpeedData *heuristic_speed_data_ = nullptr;

    LcDirection lc_direction_;
    double cross_point_l_;
    std::vector<double> left_bound_;
    std::vector<double> right_bound_;

    double boundary_buff_ = 0;
    double vehicle_length_ = 4.933;

    double vehicle_width_ = 1.98;
    double longitudinal_safe_buff_ = 1.0;

    double lateral_safe_buff_ = 0.4;
};

}  // namespace planning_lib
}  // namespace jarvis