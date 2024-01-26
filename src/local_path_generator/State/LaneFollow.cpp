#include "LaneFollow.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {

bool LaneFollowState::Process(const PosFromIns &vehicle_state,
                              const RoutingMsg &routing_msg,
                              const PnCLane &pnc_lanes,
                              const Injector &injector,
                              const DecideStatus &decide_status,
                              PnCLane &pnc_center_lanes,
                              LaneChangeMsg &lane_change_msg) {
    cos_trans_theta_ = injector.cos_trans_theta;
    sin_trans_theta_ = injector.sin_trans_theta;
    // SG_INFO("lane follow state");
    pnc_center_lanes.lanes = pnc_lanes.lanes;
    pnc_center_lanes.self_car_lane_cnt = pnc_lanes.self_car_lane_cnt;

    for (int32_t i = 0; i < pnc_lanes.lanes.size(); ++i) {
        std::vector<PathPoint> center_line;
        for (float l = 0; l < lane_length_; l += 1.0) {
            float s = 0;
            for (int32_t j = 0; j < pnc_lanes.lanes[i].fit_param.size(); ++j) {
                s += pnc_lanes.lanes[i].fit_param[j] * std::pow(l, j);
            }
            // //SG_INFO("s = %f,l = %f", s, l);

            double enu_x = s * cos_trans_theta_ + l * sin_trans_theta_ +
                           vehicle_state.position.x;
            double enu_y = -s * sin_trans_theta_ + l * cos_trans_theta_ +
                           vehicle_state.position.y;

            PathPoint point;
            point.position_m.x = enu_x;
            point.position_m.y = enu_y;
            point.position_m.z = 0;
            point.theta_rad = 0;
            point.s_m = l;
            point.dkappa = ref_vel_;
            center_line.emplace_back(point);
        }
        pnc_center_lanes.center_lines.emplace_back(std::move(center_line));
    }
    IsNeedLineChange(vehicle_state, routing_msg, pnc_lanes, decide_status,
                     lane_change_msg);

    return true;
}
void LaneFollowState::IsNeedLineChange(const PosFromIns &vehicle_state,
                                       const RoutingMsg &routing_msg,
                                       const PnCLane &pnc_lanes,
                                       const DecideStatus &decide_status,
                                       LaneChangeMsg &lane_change_msg) {
    if (pnc_lanes.lanes.size() <= 1) {
        return;
    }
    if (pre_lane_change_msg.is_lane_change == true &&
        (decide_status == DecideStatus::ROUTE_CHANGE_LEFT ||
         decide_status == DecideStatus::ROUTE_CHANGE_RIGHT)) {
        bool has_target_id = false;
        bool has_current_id = false;
        int32_t target_cnt = -1;
        for (int32_t i = 0; i < pnc_lanes.lanes.size(); ++i) {
            if (pnc_lanes.lanes[i].id == pre_lane_change_msg.target_lane_id) {
                has_target_id = true;
                target_cnt = i;
            }
            if (pnc_lanes.lanes[i].id == pre_lane_change_msg.curr_lane_id) {
                has_current_id = true;
            }
        }
        if (has_target_id == false) {
            pre_lane_change_msg.is_lane_change = false;
        } else {
            if (has_current_id == false) {
                target_cnt = target_cnt > 0 ? target_cnt - 1 : 0;
                pre_lane_change_msg.curr_lane_id =
                    pnc_lanes.lanes[target_cnt].id;
            }
        }
        lane_change_msg = pre_lane_change_msg;
    }

    // SG_INFO("self_car_lane_cnt = %d,size = %d", pnc_lanes.self_car_lane_cnt,
    // pnc_lanes.lanes.size());

    lane_change_msg.curr_lane_id =
        pnc_lanes.lanes[pnc_lanes.self_car_lane_cnt].id;
    if (routing_msg.turn == TURN::LEFT) {
        if (pnc_lanes.self_car_lane_cnt < pnc_lanes.lanes.size() - 1) {
            lane_change_msg.is_lane_change = true;
            lane_change_msg.turn = TURN::LEFT;
            lane_change_msg.target_lane_id =
                pnc_lanes.lanes[pnc_lanes.self_car_lane_cnt + 1].id;
        }
    } else if (routing_msg.turn == TURN::STRAIGHT) {
        // if (pnc_lanes.self_car_lane_cnt == pnc_lanes.lanes.size() - 1) {
        //     lane_change_msg.is_lane_change = true;
        //     lane_change_msg.turn = TURN::RIGHT;
        //     lane_change_msg.target_lane_id =
        //         pnc_lanes.lanes[pnc_lanes.self_car_lane_cnt - 1].id;
        // }
        if (pnc_lanes.lanes.size() < 3) {
            if (pnc_lanes.self_car_lane_cnt == 0) {
                lane_change_msg.is_lane_change = false;
                lane_change_msg.turn = TURN::STRAIGHT;
                lane_change_msg.target_lane_id =
                    pnc_lanes.lanes[pnc_lanes.self_car_lane_cnt + 1].id;
            }
        }
        if (pnc_lanes.lanes.size() >= 3) {
            if (pnc_lanes.self_car_lane_cnt == 0) {
                lane_change_msg.is_lane_change = true;
                lane_change_msg.turn = TURN::LEFT;
                lane_change_msg.target_lane_id =
                    pnc_lanes.lanes[pnc_lanes.self_car_lane_cnt + 1].id;
            }
        }
    } else if (routing_msg.turn == TURN::RIGHT) {
        if (pnc_lanes.self_car_lane_cnt != 0) {
            lane_change_msg.is_lane_change = true;
            lane_change_msg.turn = TURN::RIGHT;
            lane_change_msg.target_lane_id =
                pnc_lanes.lanes[pnc_lanes.self_car_lane_cnt - 1].id;
        }
    }
    pre_lane_change_msg = lane_change_msg;
    // //SG_INFO("Is neeed line chage");
    // TurnPrint(lane_change_msg.turn);
}

}  // namespace planning_lib
}  // namespace jarvis