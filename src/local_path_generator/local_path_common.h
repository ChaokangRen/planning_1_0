#pragma once
#include "common.h"
#include "planning_interface.h"

namespace jarvis {
namespace planning_lib {

constexpr int32_t priority_0 = 0;
constexpr int32_t priority_1 = 1;
constexpr int32_t priority_2 = 2;
constexpr double sidewalk_width = 3.0;
constexpr double half_lane_width_standard = 1.75;
constexpr double bicycle_lane_width = 3.0;
constexpr double lane_width_standard = 3.5;
constexpr double traffic_light_to_junction_dist = 4.0;
constexpr double right_turn_lat = 16.0;
constexpr double right_turn_lon = 21.0;

enum class DriveState {
    LaneFollow = 0,
    InterJunction = 1,
    OnJunction = 2,
    AwayJunction = 3
};

typedef struct {
    double cos_trans_theta = 0.0;
    double sin_trans_theta = 0.0;

    double stop_line_dist_eastimate = 0.0;
    double traffic_light_dist = 0.0;

    double left_road_width_estimate = 0.0;

    double lateral_road_width = 0.0;

    bool has_stable_slope = false;

    double stable_slope = 0.0;

} Injector;

void SearchNearestCurve(const PosFromIns &vehicle_state,
                        const std::vector<Point3d> &left_curvature_line,
                        const std::vector<Point3d> &lateral_extended_line,
                        std::vector<Point3d> &curvature_line);

void SearchNearestLaneLineCount(const CenterLine &lane_line, const double y,
                                double &x, int32_t &count);
}  // namespace planning_lib
}  // namespace jarvis