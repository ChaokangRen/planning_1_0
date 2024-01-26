#include "center_line_generator.h"
#include "common.h"
#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

void LandMarkToCenterLine(std::vector<FitCurve> &center_line_fit) {}

inline void SetCenterType(const std::vector<LaneLineType> &lane_lines_type,
                          const int32_t i, FitCurve &center_line_fit_curve) {
    // //SG_INFO("i = %d,type1 = %d,type2 = %d", i, lane_lines_type[i],
    //         lane_lines_type[i + 1]);
    if (i >= lane_lines_type.size()) {
        return;
    }
    if (lane_lines_type[i] == LaneLineType::BROKEN) {
        center_line_fit_curve.can_right_change = true;
        // //SG_INFO("can turn right");
    } else {
        center_line_fit_curve.can_right_change = false;
        // //SG_INFO("can't turn right");
    }
    if (i >= lane_lines_type.size() - 1) {
        return;
    }
    if (lane_lines_type[i + 1] == LaneLineType::BROKEN) {
        center_line_fit_curve.can_left_change = true;
        // //SG_INFO("can turn left");
    } else {
        center_line_fit_curve.can_left_change = false;
        // //SG_INFO("can't turn left");
    }

    if (i == 0) {
        center_line_fit_curve.can_right_change = false;
    }
    if (i == lane_lines_type.size() - 1) {
        center_line_fit_curve.can_left_change = false;
    }
}
bool CenterLineGenerator::Process(
    const LaneInfo &lane_info, const PosFromIns &vehicle_state,
    const ObstaclesInfo &obstacles_msg, const RoutingMsg &routing_msg,
    const TrafficLightFrameResult &traffic_lights_results) {
    std::map<double, int32_t> lanes_pront_x_map;
    for (int32_t i = 0; i < lane_info.lane_lines_msg.size(); ++i) {
        if (lane_info.lane_lines_msg[i].type != LaneLineMsg::Type::LANE_LANE) {
            continue;
        }
        double x =
            lane_info.lane_lines_msg[i].lane_line.back().point.position_m.x;

        if (lane_info.lane_lines_msg[i].confidence < 0.3) {
            continue;
        }
        lanes_pront_x_map.insert(std::make_pair(x, i));
    }
    std::vector<int32_t> lanes_num;
    for (const auto &pront_x : lanes_pront_x_map) {
        lanes_num.emplace_back(pront_x.second);
    }
    // SG_INFO("lanes_pront_x size = %d,lanes_num size = %d",
    // lanes_pront_x_map.size(), lanes_num.size());

    std::vector<std::vector<Vec2d>> lane_lines_vec2d;
    std::vector<LaneLineType> lane_lines_type;
    std::vector<int32_t> lane_lines_id;
    for (int32_t i = lanes_num.size() - 1; i >= 0; --i) {
        // if (lane_info.lane_lines_msg[lanes_num[i]]
        //         .lane_line.back()
        //         .point.position_m.x < -6.0) {
        //     break;
        // }
        // //SG_INFO("i = %d,type = %d", i,
        //         lane_info.lane_lines_msg[lanes_num[i]].lane_line_type);
        std::vector<Vec2d> lines;
        for (int32_t j = 0;
             j < lane_info.lane_lines_msg[lanes_num[i]].lane_line.size();
             j += 10) {
            double x = lane_info.lane_lines_msg[lanes_num[i]]
                           .lane_line[j]
                           .point.position_m.x;
            double y = lane_info.lane_lines_msg[lanes_num[i]]
                           .lane_line[j]
                           .point.position_m.y;
            lines.emplace_back(Vec2d(y, x));
        }
        lane_lines_vec2d.emplace_back(std::move(lines));
        lane_lines_type.emplace_back(
            lane_info.lane_lines_msg[lanes_num[i]].lane_line_type);
        lane_lines_id.emplace_back(
            lane_info.lane_lines_msg[lanes_num[i]].lane_id);
    }

    std::vector<FitCurve> fit_curves;
    double slope = 0.0;
    // SG_INFO("lane_lines_vec2d size = %d", lane_lines_vec2d.size());
    for (int32_t i = 0; i < lane_lines_vec2d.size(); ++i) {
        if (lane_lines_vec2d[i].size() < 2) {
            continue;
        }
        FitCurve fit_curve;
        LeastSquareFitCurve(lane_lines_vec2d[i], fit_curve_order_, fit_curve);
        fit_curve.id = std::to_string(lane_lines_id[i]);
        slope += fit_curve.fit_param[1];
        fit_curves.emplace_back(std::move(fit_curve));
    }

    slope = slope / lane_lines_vec2d.size();
    // SG_INFO("slope = %lf", slope);
    // SG_INFO("fit curve size = %d", fit_curves.size());

    std::vector<FitCurve> center_line_fit;
    if (fit_curves.size() == 0) {
        PnCLaneAddStarightLine(pnc_lanes);
        return false;
    }
    double lane_theta = -3.708;
    double theta_error = vehicle_state.yaw - lane_theta;

    // for (int32_t i = i; i < fit_curves.size(); ++i) {
    //     double b = fit_curves[i].fit_param[0] *
    //     std::cos(theta_error);
    //     //SG_INFO("i = %d,b= %lf,b2 = %lf,theta = %f", i, b,
    //             fit_curves[i].fit_param[0], theta_error);
    // }
    for (int32_t i = 0; i < fit_curves.size() - 1; ++i) {
        const double &k = fit_curves[i].fit_param[1];
        const double &b = fit_curves[i].fit_param[0];
        const double &k1 = fit_curves[i + 1].fit_param[1];
        const double &b1 = fit_curves[i + 1].fit_param[0];
        // y0 = 0
        double dist = std::fabs(b1 - b) / std::sqrt(1 + k * k);
        // //SG_INFO("dist = %lf,k = %lf,b = %lf,k1 = %lf,b1 = %lf",
        // dist, k, b, k1, b1);
        FitCurve center_line_fit_curve;
        center_line_fit_curve.id = fit_curves[i].id + fit_curves[i + 1].id;
        if (dist < 1.0) {
            continue;
        } else if (dist >= 1.0 && dist < 4.0) {
            center_line_fit_curve.fit_param.emplace_back(b - (b - b1) / 2);
            center_line_fit_curve.fit_param.emplace_back(k);

            SetCenterType(lane_lines_type, i, center_line_fit_curve);

            center_line_fit.emplace_back(center_line_fit_curve);
            // has one center line
        } else if (dist >= 4.2 && dist < 5.5) {
            double left_offset = b - center_line_offset_;
            double right_offset = b1 + center_line_offset_;
            if (std::fabs(left_offset) < std::fabs(right_offset)) {
                center_line_fit_curve.fit_param.emplace_back(left_offset);
                center_line_fit_curve.fit_param.emplace_back(k);
            } else {
                center_line_fit_curve.fit_param.emplace_back(right_offset);
                center_line_fit_curve.fit_param.emplace_back(k1);
            }
            SetCenterType(lane_lines_type, i, center_line_fit_curve);
            center_line_fit.emplace_back(center_line_fit_curve);
        } else if (dist >= 5.5 && dist < 8.0) {
            double left_offset = b - (b - b1) / 3;
            double right_offset = b - 2 * (b - b1) / 3;
            center_line_fit_curve.fit_param =
                std::vector<double>{left_offset, k};

            SetCenterType(lane_lines_type, i, center_line_fit_curve);
            center_line_fit_curve.can_left_change = true;
            center_line_fit.emplace_back(center_line_fit_curve);

            center_line_fit_curve.fit_param =
                std::vector<double>{right_offset, k1};
            SetCenterType(lane_lines_type, i, center_line_fit_curve);
            center_line_fit_curve.can_right_change = true;
            center_line_fit.emplace_back(center_line_fit_curve);
        } else {
            double center_slope = (k + k1) / 2;
            int32_t cnt = 0;
            for (float offset = center_line_offset_; offset < dist;
                 offset += center_line_offset_) {
                center_line_fit_curve.fit_param =
                    std::vector<double>{b - offset, center_slope};
                if (cnt == 0) {
                    SetCenterType(lane_lines_type, i, center_line_fit_curve);
                    center_line_fit_curve.can_left_change = true;
                } else {
                    center_line_fit_curve.can_left_change = true;
                    center_line_fit_curve.can_right_change = true;
                }
                ++cnt;
                center_line_fit.emplace_back(center_line_fit_curve);
            }
            if (lane_lines_type[i + 1] == LaneLineType::BROKEN) {
                center_line_fit.back().can_right_change = true;
            } else {
                center_line_fit.back().can_right_change = false;
            }
        }
    }
    // //SG_INFO("center line fit = %d", center_line_fit.size());
    // for (int32_t i = 0; i < center_line_fit.size(); ++i) {
    //     //SG_INFO("i = %d,k = %lf,b = %lf,id = %s", i,
    //             center_line_fit[i].fit_param[1],
    //             center_line_fit[i].fit_param[0],
    //             center_line_fit[i].id.c_str());
    // }
    PnCLanesGenerator(center_line_fit, vehicle_state);

    pre_vehicle_pos.set_x(vehicle_state.position.x);
    pre_vehicle_pos.set_y(vehicle_state.position.y);

    return true;
}

int32_t CenterLineGenerator::PnCLanesGenerator(
    const std::vector<FitCurve> &center_line_fit,
    const PosFromIns &vehicle_state) {
    int32_t lane_line_num = 1;

    int32_t self_car_lane_cnt = -1;

    for (int32_t i = 0; i < center_line_fit.size(); ++i) {
        if (center_line_fit[i].fit_param[0] < 0) {
            break;
        }
        self_car_lane_cnt = i;
    }

    if (self_car_lane_cnt < center_line_fit.size() - 1) {
        if (std::fabs(center_line_fit[self_car_lane_cnt].fit_param[0]) >
            std::fabs(center_line_fit[self_car_lane_cnt + 1].fit_param[0])) {
            ++self_car_lane_cnt;
        }
    }
    int32_t left_lane_cnt = self_car_lane_cnt;
    int32_t right_lane_cnt = self_car_lane_cnt;
    for (int32_t i = self_car_lane_cnt; i < center_line_fit.size(); ++i) {
        if (center_line_fit[i].can_left_change == true) {
            ++lane_line_num;
            left_lane_cnt = i + 1;
        } else {
            break;
        }
    }
    if (left_lane_cnt >= center_line_fit.size()) {
        left_lane_cnt = center_line_fit.size() - 1;
    }
    for (int32_t i = self_car_lane_cnt; i >= 0; --i) {
        if (center_line_fit[i].can_right_change == true) {
            ++lane_line_num;
            right_lane_cnt = i - 1;
        } else {
            break;
        }
    }
    if (right_lane_cnt < 0) {
        right_lane_cnt = 0;
    }

    if (lane_line_num >= 5) lane_line_num = 5;
    // for (int32_t i = 0; i < center_line_fit.size(); ++i) {
    //     if (center_line_fit[i].can_left_change == true &&
    //         center_line_fit[i].can_right_change == true) {
    //         //SG_INFO("i = %d,k = %lf,b = %lf,can turn left,can turn right",
    //         i,
    //                 center_line_fit[i].fit_param[1],
    //                 center_line_fit[i].fit_param[0]);
    //     } else if (center_line_fit[i].can_left_change == false &&
    //                center_line_fit[i].can_right_change == true) {
    //         //SG_INFO("i = %d,k = %lf,b = %lf,can't turn left,can turn right
    //         ", i,
    //                 center_line_fit[i].fit_param[1],
    //                 center_line_fit[i].fit_param[0]);
    //     } else if (center_line_fit[i].can_left_change == true &&
    //                center_line_fit[i].can_right_change == false) {
    //         //SG_INFO("i = %d,k = %lf,b = %lf,can turn left,can't turn right
    //         ", i,
    //                 center_line_fit[i].fit_param[1],
    //                 center_line_fit[i].fit_param[0]);
    //     } else if (center_line_fit[i].can_left_change == false &&
    //                center_line_fit[i].can_right_change == false) {
    //         //SG_INFO("i = %d,k = %lf,b = %lf,can't turn
    //         left,can'tturnright", i,
    //                 center_line_fit[i].fit_param[1],
    //                 center_line_fit[i].fit_param[0]);
    //     }
    // }

    int32_t max_probability_lane_cnt = LaneNumListInsert(lane_line_num);
    // SG_INFO("max_probability_lane_cnt = %d,lane_line_num = %d",
    // max_probability_lane_cnt, lane_line_num);
    // SG_INFO("left cnt = %d,right cnt = %d,slef_cnt = %d", left_lane_cnt,
    // right_lane_cnt, self_car_lane_cnt);

    // if (max_probability_lane_cnt == lane_line_num) {
    //     pnc_lanes.lanes.clear();

    //     for (int32_t i = right_lane_cnt; i <= left_lane_cnt; ++i) {
    //         pnc_lanes.lanes.emplace_back(center_line_fit[i]);
    //     }

    //     update_cnt = 0;
    //     vehicle_move_distance_ = 0.0;
    // } else {
    //     ++update_cnt;
    //     double dx = vehicle_state.position.x - pre_vehicle_pos.x();
    //     double dy = vehicle_state.position.y - pre_vehicle_pos.y();
    // }
    pnc_lanes.lanes.clear();
    for (int32_t i = right_lane_cnt; i <= left_lane_cnt; ++i) {
        pnc_lanes.lanes.emplace_back(center_line_fit[i]);
    }
    PnCLaneAddStarightLine(pnc_lanes);
    // for (int32_t i = 0; i < pnc_lanes.lanes.size(); ++i) {
    //     //SG_INFO("self id = %s,lane id = %s", self_lane_id_.c_str(),
    //             pnc_lanes.lanes[i].id.c_str());
    // }

    double lane_nearest_dist = 999;

    bool has_pnc_lane_id = false;

    for (int32_t i = 0; i < pnc_lanes.lanes.size(); ++i) {
        if (self_lane_id_ == pnc_lanes.lanes[i].id) {
            pnc_lanes.self_car_lane_cnt = i;
            has_pnc_lane_id = true;
            // SG_WARN("has pre id = %s", self_lane_id_.c_str());
            break;
        }
    }

    if (has_pnc_lane_id == false ||
        std::fabs(pnc_lanes.lanes[pnc_lanes.self_car_lane_cnt].fit_param[0]) >
            1.0) {
        // SG_WARN("no has pre id");
        for (int32_t i = 0; i < pnc_lanes.lanes.size(); ++i) {
            if (lane_nearest_dist >
                std::fabs(pnc_lanes.lanes[i].fit_param[0])) {
                lane_nearest_dist = std::fabs(pnc_lanes.lanes[i].fit_param[0]);
                pnc_lanes.self_car_lane_cnt = i;
                self_lane_id_ = pnc_lanes.lanes[i].id;
            }
        }
    }

    return -1;
}

int32_t CenterLineGenerator::LaneNumListInsert(const int32_t lane_num) {
    static std::vector<int32_t> lane_num_vec(5, 0);
    lane_num_list.push_back(lane_num);
    if (lane_num_list.size() > list_max_size_) {
        --lane_num_vec[lane_num_list.front()];
        lane_num_list.pop_front();
    }

    ++lane_num_vec[lane_num];

    int32_t max_probability_lane_num = 0;
    int32_t max_num = -1;
    for (int32_t i = 0; i < lane_num_vec.size(); ++i) {
        if (lane_num_vec[i] > max_num) {
            max_num = lane_num_vec[i];
            max_probability_lane_num = i;
        }
    }
    return max_probability_lane_num;
}

}  // namespace planning_lib
}  // namespace jarvis