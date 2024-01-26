#include "local_path_generator.h"

#include <map>

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {
void LocalPathGenerator::Process(
    const LaneInfo &lane_info, const PosFromIns &vehicle_state,
    const ObstaclesInfo &obstacles_msg,
    const TrafficLightFrameResult &traffic_lights_results,
    const RoutingMsg &routing_msg, const DecideStatus &decide_status,
    PnCLane &center_lines, LaneChangeMsg &lane_change_msg) {
    center_line_generator_.Process(lane_info, vehicle_state, obstacles_msg,
                                   routing_msg, traffic_lights_results);
    // center_lines_ = lane_info.center_line;
    if (lane_info.lane_lines_msg.empty() == false) {
        RightRoadWidthEstimate(lane_info);
    }
    if (turn_lock_ == false) {
        routing_msg_ = routing_msg;
    }
    // drive_state_ = DriveState::LaneFollow;

    // routing_msg_.turn = TURN::LEFT;

    double trans_theta = (2.5 * M_PI - vehicle_state.yaw);
    double sin_trans_theta = std::sin(trans_theta);
    double cos_trans_theta = std::cos(trans_theta);

    StopLineDistEstimate(lane_info, vehicle_state);
    UpdateHasTrafficLight(traffic_lights_results);
    CheckStableLaneSlope(lane_info, vehicle_state);

    injector_.cos_trans_theta = cos_trans_theta;
    injector_.sin_trans_theta = sin_trans_theta;
    injector_.stop_line_dist_eastimate = stop_line_dist_eastimate_;
    injector_.traffic_light_dist = traffic_light_dist_;
    injector_.has_stable_slope = has_stable_slope_;
    injector_.stable_slope = stable_slope_;
    injector_.left_road_width_estimate = left_road_width_estimate_;

    StateMachineProcess(lane_info, vehicle_state, obstacles_msg,
                        traffic_lights_results, routing_msg_, decide_status,
                        center_lines, lane_change_msg);
}

bool LocalPathGenerator::HasRoadJunctionIn50m(const RoutingMsg &routing_msg) {
    if (stop_line_dist_eastimate_ > 10 && stop_line_dist_eastimate_ < 50 &&
        traffic_light_dist_ < 90) {
        return true;
    }
    if (traffic_light_dist_ < 90 && routing_msg.distance < 50 &&
        routing_msg.turn != TURN::NONE) {
        return true;
    }
    if (stop_line_dist_eastimate_ < 40 && stop_line_dist_eastimate_ > 10 &&
        routing_msg.distance < 50 && routing_msg.turn != TURN::NONE) {
        // SG_WARN("stop_line_dist_eastimate_ = %lf,routing_msg.distance = %lf",
        //         stop_line_dist_eastimate_, routing_msg.distance);
        // TurnPrint(routing_msg.turn);
        return true;
    }
    return false;
}

void LocalPathGenerator::StateMachineProcess(
    const LaneInfo &lane_info, const PosFromIns &vehicle_state,
    const ObstaclesInfo &obstacles_msg,
    const TrafficLightFrameResult &traffic_lights_results,
    const RoutingMsg &routing_msg, const DecideStatus &decide_status,
    PnCLane &center_lines, LaneChangeMsg &lane_change_msg) {
    std::vector<Point3d> curvature_line;

    PnCLane pnc_lanes = center_line_generator_.GetPnCLine();

    if (drive_state_ == DriveState::LaneFollow) {
        // SG_INFO("LaneFollow");
        lane_follow_state_.Process(vehicle_state, routing_msg, pnc_lanes,
                                   injector_, decide_status, center_lines,
                                   lane_change_msg);
        if (is_need_reset_ == true) {
            Reset();
        }
        if (HasRoadJunctionIn50m(routing_msg)) {
            drive_state_ = DriveState::InterJunction;
        }
        turn_lock_ = false;
        is_need_reset_ = false;
    } else if (drive_state_ == DriveState::InterJunction) {
        // SG_INFO("InterJunction");
        inter_junction_state_.Process(lane_info, vehicle_state, obstacles_msg,
                                      pnc_lanes, routing_msg, injector_,
                                      curvature_line);
        PostProcessCurveLine(curvature_line, center_lines);
        lane_change_msg.is_lane_change = false;

        left_curvature_line_ = curvature_line;
        lateral_extended_line_ = inter_junction_state_.GetExtendLine();
        turn_lock_ = true;

        if (stop_line_dist_eastimate_ <= 6 && has_traffic_light_ == true) {
            drive_state_ = DriveState::OnJunction;
        }

        inter_junction_yaw_ = vehicle_state.yaw;
        is_need_reset_ = true;
    } else if (drive_state_ == DriveState::OnJunction) {
        // SG_INFO("OnJunction");
        on_junction_state_.OnJunctionCurve(vehicle_state, left_curvature_line_,
                                           lateral_extended_line_,
                                           curvature_line);
        PostProcessCurveLine(curvature_line, center_lines);
        lane_change_msg.is_lane_change = false;

        turn_lock_ = true;
        if (routing_msg.turn == TURN::LEFT || routing_msg.turn == TURN::RIGHT) {
            if (std::fabs(vehicle_state.yaw - inter_junction_yaw_) >
                40 * deg_to_rad) {
                drive_state_ = DriveState::AwayJunction;
            }
        } else if (routing_msg.turn == TURN::STRAIGHT) {
            if (has_stable_slope_) {
                drive_state_ = DriveState::LaneFollow;
            }
        }
        if (routing_msg.turn == TURN::STRAIGHT) {
            drive_state_ = DriveState::AwayJunction;
        }
        stable_slope_cnt_ = 0;
        is_need_reset_ = true;
    } else if (drive_state_ == DriveState::AwayJunction) {
        // SG_INFO("AwayJunction");
        away_junction_state_.AwayJunctionCurve(
            vehicle_state, left_curvature_line_, injector_, routing_msg,
            lateral_extended_line_, curvature_line);
        PostProcessCurveLine(curvature_line, center_lines);
        lane_change_msg.is_lane_change = false;
        turn_lock_ = true;
        if (stable_slope_cnt_ >= 10) {
            drive_state_ = DriveState::LaneFollow;
        }
        is_need_reset_ = true;
    }
    pre_curvature_line_ = curvature_line;
}
void LocalPathGenerator::RightRoadWidthEstimate(const LaneInfo &lane_info) {
    std::map<double, int32_t> lane_map;
    for (int32_t i = 0; i < lane_info.lane_lines_msg.size(); ++i) {
        lane_map.insert(std::make_pair(
            lane_info.lane_lines_msg[i].lane_line.front().point.position_m.x,
            i));
    }
    std::vector<int32_t> lane_proj_vec;
    int32_t self_car_lane_num = -1;
    for (auto &ite : lane_map) {
        lane_proj_vec.emplace_back(ite.second);
        if (ite.first < 0) {
            ++self_car_lane_num;
        }
    }
    int32_t current_frame_lane_numbers_estimate = 0;
    double current_frame_road_with_estimate = 0;
    if (self_car_lane_num == -1) {
        current_frame_lane_numbers_estimate =
            lane_info.lane_lines_msg.size() - 1;
        current_frame_road_with_estimate =
            lane_info.lane_lines_msg[lane_proj_vec.back()]
                .lane_line.front()
                .point.position_m.x -
            lane_info.lane_lines_msg[lane_proj_vec.front()]
                .lane_line.front()
                .point.position_m.x;
    } else {
        int32_t left_solid_line_num = self_car_lane_num;
        int32_t right_solid_line_num = self_car_lane_num;

        for (int32_t i = self_car_lane_num; i >= 0; --i) {
            left_solid_line_num = i;
            const LaneLineType &lane_type =
                lane_info.lane_lines_msg[lane_proj_vec[i]].lane_line_type;
            if (lane_type == LaneLineType::SOLID ||
                lane_type == LaneLineType::BROKEN_SOLID ||
                lane_type == LaneLineType::SOLID_SOLID) {
                break;
            }
        }
        for (int32_t i = self_car_lane_num + 1; i < lane_proj_vec.size(); ++i) {
            right_solid_line_num = i;
            const LaneLineType &lane_type =
                lane_info.lane_lines_msg[lane_proj_vec[i]].lane_line_type;
            if (lane_type == LaneLineType::SOLID ||
                lane_type == LaneLineType::BROKEN_SOLID ||
                lane_type == LaneLineType::SOLID_SOLID) {
                break;
            }
        }
        current_frame_lane_numbers_estimate =
            right_solid_line_num - left_solid_line_num;
        current_frame_road_with_estimate =
            lane_info.lane_lines_msg[lane_proj_vec[right_solid_line_num]]
                .lane_line.front()
                .point.position_m.x -
            lane_info.lane_lines_msg[lane_proj_vec[left_solid_line_num]]
                .lane_line.front()
                .point.position_m.x;
    }
    right_lane_numbers_estimate_ = std::max(current_frame_lane_numbers_estimate,
                                            right_lane_numbers_estimate_);
    right_road_width_estimate_ =
        std::fmax(current_frame_road_with_estimate, right_road_width_estimate_);

    left_road_width_estimate_ = right_road_width_estimate_ + sidewalk_width +
                                half_lane_width_standard + bicycle_lane_width;
}

void LocalPathGenerator::UpdateHasStopLine(const LaneInfo &lane_info) {
    bool current_has_stop_line = false;
    stop_line_msg_ptr_ = nullptr;
    for (const LaneLineMsg &lane_line_msg : lane_info.lane_lines_msg) {
        if (lane_line_msg.type == LaneLineMsg::Type::STOP_LINE) {
            if (pre_has_stop_line_ == true) {
                ++stop_line_count_;
            }
            current_has_stop_line = true;
        }
    }
    if (pre_has_stop_line_ == false && current_has_stop_line == false) {
        stop_line_count_ = 0;
        has_stop_line_ = false;
    }
    pre_has_stop_line_ = current_has_stop_line;

    if (stop_line_count_ >= 5) {
        has_stop_line_ = true;
    }
}
void LocalPathGenerator::StopLineDistEstimate(const LaneInfo &lane_info,
                                              const PosFromIns &vehicle_state) {
    UpdateHasStopLine(lane_info);
    std::vector<LaneLineMsg const *> stop_line_vec;
    for (const LaneLineMsg &lane_line_msg : lane_info.lane_lines_msg) {
        if (lane_line_msg.type == LaneLineMsg::Type::STOP_LINE) {
            stop_line_vec.emplace_back(&lane_line_msg);
        }
    }
    double vehicle_speed =
        std::sqrt(vehicle_state.velocity.x * vehicle_state.velocity.x +
                  vehicle_state.velocity.y * vehicle_state.velocity.y);

    if (drive_state_ != DriveState::LaneFollow &&
        stop_line_dist_eastimate_ < 10) {
        stop_line_dist_eastimate_ -= vehicle_speed * planning_period_;
        return;
    }
    if (stop_line_vec.empty() == false && has_stop_line_ == true) {
        double current_stop_line_dist_min =
            stop_line_vec.front()->lane_line.front().point.position_m.y;
        // SG_INFO("stop line x = %lf", current_stop_line_dist_min);
        for (int32_t i = 1; i < stop_line_vec.size(); ++i) {
            // SG_INFO("current_stop_line_dist_min = %lf,2 = %lf",
            // current_stop_line_dist_min,
            // stop_line_vec[i]->lane_line.front().point.position_m.y);
            if (current_stop_line_dist_min >
                stop_line_vec[i]->lane_line.front().point.position_m.y) {
                current_stop_line_dist_min =
                    stop_line_vec[i]->lane_line.front().point.position_m.y;
            }
        }

        if (stop_line_dist_eastimate_ < 0) {
            stop_line_dist_eastimate_ = current_stop_line_dist_min;
        } else {
            double stop_line_error = std::fabs(stop_line_dist_eastimate_ -
                                               current_stop_line_dist_min);
            //  SG_INFO(
            //     "current_stop_line_dist_min = %lf,stop_line_dist_eastimate_ =
            //     "
            //     "%lf,stop_line_error = %lf",
            //     current_stop_line_dist_min, stop_line_dist_eastimate_,
            //     stop_line_error);

            if (current_stop_line_dist_min < 40 && stop_line_error < 20) {
                stop_line_dist_eastimate_ = current_stop_line_dist_min;
            } else {
                stop_line_dist_eastimate_ -= vehicle_speed * planning_period_;
            }
        }
    } else {
        ++stop_line_disappear_cnt;
        if (stop_line_dist_eastimate_ > 0) {
            stop_line_dist_eastimate_ -= vehicle_speed * planning_period_;
        }
        if (stop_line_disappear_cnt >= 10 && stop_line_dist_eastimate_ > 20) {
            stop_line_dist_eastimate_ = -1;
            stop_line_disappear_cnt = 0;
        }
    }
    // SG_INFO("stop_line_dist_eastimate_ = %lf", stop_line_dist_eastimate_);
}

void LocalPathGenerator::UpdateHasTrafficLight(
    const TrafficLightFrameResult &traffic_lights_results) {
    bool current_has_traffic_light = false;
    traffic_light_semantic_ptr_ = nullptr;
    if (traffic_lights_results.semantic_straight.color !=
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_DARK &&
        traffic_lights_results.semantic_straight.color !=
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN) {
        current_has_traffic_light = true;
        traffic_light_semantic_ptr_ = &traffic_lights_results.semantic_straight;
    }
    if (traffic_lights_results.semantic_left.color !=
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_DARK &&
        traffic_lights_results.semantic_left.color !=
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN) {
        current_has_traffic_light = true;
        traffic_light_semantic_ptr_ = &traffic_lights_results.semantic_left;
    }
    if (traffic_lights_results.semantic_right.color !=
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_DARK &&
        traffic_lights_results.semantic_right.color !=
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN) {
        current_has_traffic_light = true;
        traffic_light_semantic_ptr_ = &traffic_lights_results.semantic_right;
    }
    if (pre_has_traffic_light_ == true && current_has_traffic_light == true) {
        ++traffic_light_count_;
    }
    pre_has_traffic_light_ = current_has_traffic_light;
    if (traffic_light_count_ >= 5) {
        has_traffic_light_ = true;
        if (traffic_light_semantic_ptr_ != nullptr) {
            traffic_light_dist_ = traffic_light_semantic_ptr_->distance;
        }
        if (traffic_light_dist_ < stop_line_dist_eastimate_) {
            has_traffic_light_ = false;
        }
    }

    // SG_INFO("traffic_light_dist = %lf", traffic_light_dist_);
}

//-70,-50,-30,...,50,70
std::vector<double> tan_interval{-2.7474, -1.1917, -0.5773, -0.1763,
                                 0.1763,  0.5773,  1.1917,  2.7474};
void LocalPathGenerator::CheckStableLaneSlope(const LaneInfo &lane_info,
                                              const PosFromIns &vehicle_state) {
    std::vector<double> tan_val;
    std::vector<int32_t> count_interval(9, 0);
    std::vector<double> slope_interval(9, 0);
    if (lane_info.lane_lines_msg.empty() == false) {
        for (const LaneLineMsg &lane_line_msg : lane_info.lane_lines_msg) {
            if (lane_line_msg.type == LaneLineMsg::Type::LANE_LANE) {
                double dx = lane_line_msg.lane_line.front().point.position_m.x -
                            lane_line_msg.lane_line.back().point.position_m.x;
                double dy = lane_line_msg.lane_line.front().point.position_m.y -
                            lane_line_msg.lane_line.back().point.position_m.y;
                tan_val.emplace_back(dx / dy);
            }
        }
    }

    if (tan_val.size() > 1) {
        for (int32_t i = 0; i < tan_val.size(); ++i) {
            for (int32_t j = 0; j < tan_interval.size(); ++j) {
                if (tan_val[i] < tan_interval[j]) {
                    ++count_interval[j];
                    slope_interval[j] += tan_val[i];
                    break;
                }
                if (tan_val[i] >= tan_interval.back()) {
                    ++count_interval.back();
                    slope_interval.back() += tan_val[i];
                }
            }
        }
        int32_t cnt_max = 0;
        int32_t cnt = 0;
        for (int32_t i = 0; i < count_interval.size(); ++i) {
            if (cnt_max < count_interval[i]) {
                cnt_max = count_interval[i];
                cnt = i;
            }
        }
        if (cnt_max <= 2) {
            has_stable_slope_ = false;
        } else {
            has_stable_slope_ = true;
            stable_slope_ = slope_interval[cnt] / cnt_max;
        }
    } else {
        has_stable_slope_ = false;
    }
    if (pre_has_stable_slope_ && has_stable_slope_) {
        ++stable_slope_cnt_;
    } else {
        stable_slope_cnt_ = 0;
    }
    pre_has_stable_slope_ = has_stable_slope_;
}

void LocalPathGenerator::PostProcessCurveLine(
    const std::vector<Point3d> &curvature_line, PnCLane &center_lines) {
    double dist_len = 0;
    int32_t np = 1;
    std::vector<PathPoint> path_line;
    int32_t local_points_num = 91;
    double delta_s = 1.0;
    for (int32_t i = 1; i < curvature_line.size(); ++i) {
        double dx = curvature_line[i].x - curvature_line[i - 1].x;
        double dy = curvature_line[i].y - curvature_line[i - 1].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        while (dist_len + dist > np * delta_s) {
            double ref_x =
                lerp(curvature_line[i - 1].x, dist_len, curvature_line[i].x,
                     dist_len + dist, np * delta_s);
            double ref_y =
                lerp(curvature_line[i - 1].y, dist_len, curvature_line[i].y,
                     dist_len + dist, np * delta_s);
            double ref_theta = 0;
            // lerp(lane[i - 1].theta_rad, dist_len, lane[i].theta_rad,
            //      dist_len + dist, np * delta_s);

            PathPoint point;
            point.position_m.x = ref_x;
            point.position_m.y = ref_y;
            point.position_m.z = 0;
            point.theta_rad = ref_theta;
            point.s_m = np * delta_s;
            point.dkappa = ref_vel_;

            path_line.emplace_back(point);
            np++;
            if (np > local_points_num) {
                break;
            }
        }
        dist_len += dist;
        if (np > local_points_num) {
            break;
        }
    }

    if (np <= local_points_num) {
        int32_t path_size = path_line.size();
        if (path_size < 2) {
            SG_ERROR("curvature generator local path error");
        } else {
            double dy = path_line[path_size - 1].position_m.y -
                        path_line[path_size - 2].position_m.y;
            double dx = path_line[path_size - 1].position_m.x -
                        path_line[path_size - 2].position_m.x;
            double dist = std::sqrt(dx * dx + dy * dy);
            double cos_theta = dx / dist;
            double sin_theta = dy / dist;
            for (int32_t i = np; i < local_points_num; ++i) {
                PathPoint point;
                point.position_m.x = path_line[path_size - 1].position_m.x +
                                     (i - np + 1) * delta_s * cos_theta;
                point.position_m.y = path_line[path_size - 1].position_m.y +
                                     (i - np + 1) * delta_s * sin_theta;
                ;
                point.position_m.z = 0;
                point.theta_rad = 0;
                point.s_m = i * delta_s;
                point.dkappa = ref_vel_;

                path_line.emplace_back(point);
            }
        }
    }
    for (int32_t i = 1; i < path_line.size(); ++i) {
        double dx = path_line[i].position_m.x - path_line[i - 1].position_m.x;
        double dy = path_line[i].position_m.y - path_line[i - 1].position_m.y;
        path_line[i - 1].theta_rad = std::atan2(dy, dx);
        // //SG_INFO("x = %lf,y = %lf,s = %lf", path_line[i].position_m.x,
        //         path_line[i].position_m.y, path_line[i].s_m);
    }
    center_lines.center_lines.emplace_back(std::move(path_line));
    center_lines.self_car_lane_cnt = 0;
    PnCLaneAddStarightLine(center_lines);
    // SG_WARN("lanes = %d,centerline = %d", center_lines.lanes.size(),
    //         center_lines.center_lines.size());
}

LocalPathGenerator::LocalPathGenerator() {
    right_lane_numbers_estimate_ = 0;
    right_road_width_estimate_ = 0.0;
    left_road_width_estimate_ = 0.0;

    has_stop_line_ = false;
    pre_has_stop_line_ = false;
    stop_line_dist_eastimate_ = 0.0;

    stop_line_msg_ptr_ = nullptr;
    stop_line_count_ = 0;

    traffic_light_semantic_ptr_ = nullptr;
    pre_has_traffic_light_ = false;
    has_traffic_light_ = false;
    traffic_light_count_ = 0;
    traffic_light_dist_ = 999.0;

    has_stable_slope_ = false;
    stable_slope_cnt_ = 0;
    pre_has_stable_slope_ = false;
    stable_slope_ = 0.0;

    drive_state_ = DriveState::LaneFollow;

    inter_junction_yaw_ = 0.0;
    turn_lock_ = false;
    pre_curvature_line_ = std::vector<Point3d>(0, Point3d{0, 0, 0});
};
void LocalPathGenerator::Reset() {
    right_lane_numbers_estimate_ = 0;
    right_road_width_estimate_ = 0.0;
    left_road_width_estimate_ = 0.0;

    has_stop_line_ = false;
    pre_has_stop_line_ = false;
    stop_line_dist_eastimate_ = -1.0;

    stop_line_msg_ptr_ = nullptr;
    stop_line_count_ = 0;
    stop_line_disappear_cnt = 0;

    traffic_light_semantic_ptr_ = nullptr;
    pre_has_traffic_light_ = false;
    has_traffic_light_ = false;
    traffic_light_count_ = 0;
    traffic_light_dist_ = 999.0;

    has_stable_slope_ = false;
    stable_slope_cnt_ = 0;
    pre_has_stable_slope_ = false;
    stable_slope_ = 0.0;

    drive_state_ = DriveState::LaneFollow;

    lane_follow_state_.Reset();
    inter_junction_state_.Reset();

    inter_junction_yaw_ = 0.0;
    turn_lock_ = false;
    // SG_INFO("Lane Follow Reset");
}

void LocalPathGenerator::SetLeftCntRightCnt(
    const PnCLane &center_lines, const LaneChangeMsg &lane_change_msg,
    int32_t &left_cnt, int32_t &right_cnt) {
    const int32_t self_car_cnt = center_lines.self_car_lane_cnt;
    if (self_car_cnt - 1 >= 0) {
        right_cnt = self_car_cnt - 1;
    } else {
        right_cnt = -1;
    }

    if (self_car_cnt + 1 < center_lines.center_lines.size()) {
        left_cnt = self_car_cnt + 1;
    } else {
        left_cnt = -1;
    }
}

}  // namespace planning_lib
}  // namespace jarvis