#pragma once

#include <vector>

#include "common.h"
#include "obstacle.h"
#include "reference_line.h"
#include "speed_data.h"
#include "st_boundary.h"

namespace jarvis {
namespace planning_lib {
class DpSpeedOptimizer {
public:
    DpSpeedOptimizer() = default;

    bool Init(const PlannerConf &planner_conf,
              const SpeedOptimizerConf &speed_optimizer_conf,
              const VehicleParams &vehicle_params);

    bool OptimalSpeed(const std::vector<PathPoint> &path_points,
                      const std::vector<Obstacle> &obstacles,
                      const ReferenceLine &reference_line,
                      const TrajectoryPoint &init_point, SpeedData *speed_vec,
                      std::vector<StBoundary> *st_boundarys, double *speed_cost,
                      std::string &debuginfo);
    void SetLcLimitSpeed(double lc_limit_v) {
        lc_limit_speed_ = lc_limit_v;
    }

private:
    PlannerConf planner_conf_;
    SpeedOptimizerConf speed_optimizer_conf_;
    VehicleParams vehicle_params_;

    double vehicle_length_ = 0.0;
    double vehicle_width_ = 0.0;

    double lc_limit_speed_;
};
}  // namespace planning_lib
}  // namespace jarvis