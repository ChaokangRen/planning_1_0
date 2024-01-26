#pragma once
#include "common.h"
#include "local_path_common.h"
#include "planning_interface.h"
namespace jarvis {
namespace planning_lib {
class LaneFollowState {
public:
    LaneFollowState() = default;

    bool Process(const PosFromIns &vehicle_state, const RoutingMsg &routing_msg,
                 const PnCLane &pnc_lanes, const Injector &injector,
                 const DecideStatus &decide_status, PnCLane &pnc_center_lanes,
                 LaneChangeMsg &lane_change_msg);
    void Reset(void) {
        double cos_trans_theta_ = 0.0;
        double sin_trans_theta_ = 0.0;

        double lane_length_ = 90.0;

        int32_t center_line_cnt_ = 0;

        double lat_jerk_ = 0.15;

        int32_t current_id_ = -1;
    }

private:
    void IsNeedLineChange(const PosFromIns &vehicle_state,
                          const RoutingMsg &routing_msg,
                          const PnCLane &pnc_lanes,
                          const DecideStatus &decide_status,
                          LaneChangeMsg &lane_change_msg);

private:
    double cos_trans_theta_ = 0.0;
    double sin_trans_theta_ = 0.0;

    double lane_length_ = 90.1;

    int32_t center_line_cnt_ = 0;

    double lat_jerk_ = 0.15;

    int32_t current_id_ = -1;

    std::vector<Point3d> pre_curvature_line;

    double ref_vel_ = 7.0;

    LaneChangeMsg pre_lane_change_msg;
};
}  // namespace planning_lib
}  // namespace jarvis