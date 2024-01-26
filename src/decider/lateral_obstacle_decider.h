#pragma once

#include "common.h"
#include "lane.h"
#include "local_path_common.h"
#include "obstacle.h"
#include "reference_line.h"

namespace jarvis {
namespace planning_lib {

class LateralObstacleDecider {
public:
    void Process(const Lane &stop_lane, const ReferenceLine &reference_lines,
                 const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
                 const PnCLane &pnc_center_lines, const DriveState &drive_state,
                 const SLPoint &adc_point, std::vector<Obstacle> &obstacles);

private:
    double lateral_safe_dist_ = 1.2;
    double half_vehicle_width_ = 1.0;
    double safe_buff_dist_ = 0.5;

    double opposite_veh_yaw_ = 1.3089969389957472;  // 75'
};
}  // namespace planning_lib
}  // namespace jarvis