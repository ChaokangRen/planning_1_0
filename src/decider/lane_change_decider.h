#pragma once

#include <algorithm>
#include <memory>

#include "common.h"
#include "decider_config.h"
#include "obstacle.h"
#include "reference_line.h"
#include "traffic_light.h"

namespace jarvis {
namespace planning_lib {

class LaneChangeDecider {
public:
    LaneChangeDecider() = default;

    // void Init(const SpeedOptimizerConf &speed_optimizer_conf);

    void Process(const PnCLane &pnc_center_lines,
                 const std::vector<Obstacle> &obstacles,
                 const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
                 const ReferenceLine &refline,
                 std::vector<PathPoint> &path_points);
    double GetVelExp() {
        return vel_exp_;
    }

private:
    bool IsLaneChange(const PnCLane &pnc_center_lines,
                      const ReferenceLine &refline);
    void IsClearToChangeLine(const std::vector<Obstacle> obstacles,
                             const PosFromIns &pos_ins,
                             const SpeedFromVeh &speed_veh,
                             const ReferenceLine &ref,
                             std::vector<PathPoint> &path_points);
    Vec2d Obs2Ego(const PosFromIns &pos_ins, double obs_px, double obs_py);
    Vec2d Obs2Ego(const PosFromIns &pos_ins, PathPoint endp);
    bool HysteresisFilter(const double obstacle_distance,
                          const double safe_distance,
                          const double distance_buffer);
    void CalLcPoint(const PosFromIns &pos_ins,
                    std::vector<PathPoint> &path_points);
    double vel_exp_;
    SLPoint lane_change_point_;
    Vec2d end_path_point_;
};
}  // namespace planning_lib
}  // namespace jarvis