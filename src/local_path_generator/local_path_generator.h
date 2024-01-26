#pragma once

#include <Eigen/Eigen>
#include <set>
#include <vector>

#include "State/AwayJunction.h"
#include "State/InterJunction.h"
#include "State/LaneFollow.h"
#include "State/OnJunction.h"
#include "center_line_generator.h"
#include "common.h"
#include "local_path_common.h"
#include "planning_interface.h"
#include "routing.h"

namespace jarvis {
namespace planning_lib {

class LocalPathGenerator {
public:
    LocalPathGenerator();
    void Process(const LaneInfo &lane_info, const PosFromIns &vehicle_state,
                 const ObstaclesInfo &obstacles_msg,
                 const TrafficLightFrameResult &traffic_lights_results,
                 const RoutingMsg &routing_msg,
                 const DecideStatus &decide_status, PnCLane &center_lines,
                 LaneChangeMsg &lane_change_msg);

    void SetLeftCntRightCnt(const PnCLane &center_lines,
                            const LaneChangeMsg &lane_change_msg,
                            int32_t &left_cnt, int32_t &right_cnt);

    void Reset();
    DriveState GetDriveState() const {
        return drive_state_;
    }

private:
    void IsNeedLineChange(const PosFromIns &vehicle_state,
                          const RoutingMsg &routing_msg,
                          const PnCLane &pnc_lanes,
                          LaneChangeMsg &lane_change_msg);
    void RightRoadWidthEstimate(const LaneInfo &lane_info);

    void UpdateHasStopLine(const LaneInfo &lane_info);

    void StopLineDistEstimate(const LaneInfo &lane_info,
                              const PosFromIns &vehicle_state);
    void UpdateHasTrafficLight(
        const TrafficLightFrameResult &traffic_lights_results);

    void CheckStableLaneSlope(const LaneInfo &lane_info,
                              const PosFromIns &vehicle_state);

    void StateMachineProcess(
        const LaneInfo &lane_info, const PosFromIns &vehicle_state,
        const ObstaclesInfo &obstacles_msg,
        const TrafficLightFrameResult &traffic_lights_results,
        const RoutingMsg &routing_msg, const DecideStatus &decide_status,
        PnCLane &center_lines, LaneChangeMsg &lane_change_msg);

    void PostProcessCurveLine(const std::vector<Point3d> &curvature_line,
                              PnCLane &center_lines);

    bool HasRoadJunctionIn50m(const RoutingMsg &routing_msg);

private:
    int32_t right_lane_numbers_estimate_ = 0;
    double right_road_width_estimate_ = 0.0;
    double left_road_width_estimate_ = 0.0;

    bool has_stop_line_ = false;
    bool pre_has_stop_line_ = false;
    double stop_line_dist_eastimate_ = 0.0;
    double planning_period_ = 0.1;
    LaneLineMsg const *stop_line_msg_ptr_ = nullptr;
    int32_t stop_line_count_ = 0;

    TrafficLightSemantic const *traffic_light_semantic_ptr_ = nullptr;
    bool pre_has_traffic_light_ = false;
    bool has_traffic_light_ = false;
    int32_t traffic_light_count_ = 0;
    double traffic_light_dist_ = 999.0;

    bool has_stable_slope_ = false;
    int32_t stable_slope_cnt_ = 0;
    bool pre_has_stable_slope_ = false;
    double stable_slope_ = 0.0;
    int32_t stop_line_disappear_cnt = 0;

    DriveState drive_state_ = DriveState::LaneFollow;

    LaneFollowState lane_follow_state_;
    InterJunctionState inter_junction_state_;
    OnJunctionState on_junction_state_;
    AwayJunctionState away_junction_state_;

    Injector injector_;
    RoutingMsg routing_msg_;

    double inter_junction_yaw_ = 0.0;

    bool turn_lock_ = false;
    bool is_need_reset_ = false;

    std::vector<Point3d> left_curvature_line_;
    std::vector<Point3d> lateral_extended_line_;
    std::vector<Point3d> pre_curvature_line_;

    CenterLineGenerator center_line_generator_;
    std::vector<CenterLine> center_lines_;
    double ref_vel_ = 7.0;
};
}  // namespace planning_lib
}  // namespace jarvis