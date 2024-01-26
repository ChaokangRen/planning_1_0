#pragma once

#include <string>

#include "common.h"
#include "reference_line.h"
#include "speed_data.h"

namespace jarvis {
namespace planning_lib {

struct ReferenceLineInfo {
    ReferenceLine reference_line;
    std::vector<PathPoint> left_navigable_area;
    std::vector<PathPoint> right_navigable_area;
};

class PlannerInterface {
public:
    virtual ~PlannerInterface(){};

    static PlannerInterface *CreateInstance();

    virtual bool Init(const PlanningConf &planning_conf) = 0;

    virtual bool Execute(
        const Lane &lane, const std::vector<ReferenceLineInfo> &reference_lines,
        const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
        const LaneChangeMsg &lane_change_msg, const PnCLane &pnc_center_lines,
        const DriveState &drive_state, TrajectoryMsg *trajectory_msg,
        std::vector<Obstacle> &obstacle, std::string &debuginfo) = 0;
    virtual int32_t GetCurrLaneId() = 0;
};

}  // namespace planning_lib
}  // namespace jarvis