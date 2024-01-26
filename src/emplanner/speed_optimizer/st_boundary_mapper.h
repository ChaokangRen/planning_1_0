#pragma once

#include "common.h"
#include "obstacle.h"
#include "reference_line.h"
#include "st_point.h"
namespace jarvis {
namespace planning_lib {

struct StBoundaryConfig {
    double boundary_buffer = 0.1;
    double high_speed_centirc_acceleration_limit = 1.2;
    double low_speed_centric_acceleration_limit = 1.4;
    double high_speed_threshold = 20.0;
    double low_speed_threshold = 0.0;
    double minimal_kappa = 1e-5;
    double point_extension = 0.1;
    double lowest_speed = 2.5;
    uint32_t num_points_to_avg_kappa = 4;
    double static_obs_nudge_speed_ratio = 0.0;
    double dynamic_obs_nudge_speed_ration = 0.0;
    double centri_jerk_speed_coeff = 0.0;
};
class StBoundaryMapper {
public:
    StBoundaryMapper(const SLStaticBoundary adc_sl_boundary)
        : adc_sl_boundary_(adc_sl_boundary) {
        StBoundaryConfig st_boundary_config;
        is_change_lane_ = false;
    }

    bool GetStBoundaryPoints(const std::vector<PathPoint> &path_data,
                             const Obstacle &obstacle,
                             const ReferenceLine &reference_line,
                             std::vector<STPoint> *upper_points,
                             std::vector<STPoint> *lower_points) const;

    bool CheckOverlap(const PathPoint &path_point, const Box2d &obs_box,
                      const double buffer) const;

private:
    const SLStaticBoundary adc_sl_boundary_;
    const StBoundaryConfig st_boundary_config_;
    bool is_change_lane_ = false;

    double lf_ = 3.89;

    double lr_ = 1.043;

    double vehicle_length_ = 4.933;

    double vehicle_width_ = 1.98;

    double vehicle_left_edge_to_center_ = 0.93;

    double vehicle_right_edge_to_center_ = 0.93;
};
}  // namespace planning_lib
}  // namespace jarvis
