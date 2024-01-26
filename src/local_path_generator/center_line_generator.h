#pragma once
#include <map>

#include "common.h"
#include "planning_interface.h"
namespace jarvis {
namespace planning_lib {

class CenterLineGenerator {
public:
    bool Process(const LaneInfo &lane_info, const PosFromIns &vehicle_state,
                 const ObstaclesInfo &obstacles_msg,
                 const RoutingMsg &routing_msg,
                 const TrafficLightFrameResult &traffic_lights_results);

    PnCLane GetPnCLine(void) {
        return pnc_lanes;
    }

private:
    int32_t PnCLanesGenerator(const std::vector<FitCurve> &center_line_fit,
                              const PosFromIns &vehicle_state);

    int32_t LaneNumListInsert(const int32_t lane_num);

private:
    int32_t fit_curve_order_ = 2;
    float slope_limit_ = 0.15;
    float center_line_offset_ = 1.5;
    float line_length_ = 90;

    std::list<int32_t> lane_num_list;
    int32_t list_max_size_ = 50;
    PnCLane pnc_lanes;
    int32_t update_cnt = 0;
    float vehicle_move_distance_ = 0.0;
    Vec2d pre_vehicle_pos;

    std::string self_lane_id_;
};
}  // namespace planning_lib
}  // namespace jarvis