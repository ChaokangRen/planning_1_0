#include "InterJunction.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {
InterJunctionState::InterJunctionState() {
    left_obs_priority_set_ = std::vector<std::map<int32_t, int32_t>>(
        3, std::map<int32_t, int32_t>());
    right_obs_priority_set_ = std::vector<std::map<int32_t, int32_t>>(
        3, std::map<int32_t, int32_t>());
};
bool InterJunctionState::Process(const LaneInfo &lane_info,
                                 const PosFromIns &vehicle_state,
                                 const ObstaclesInfo &obstacles_msg,
                                 const PnCLane &pnc_lanes,
                                 const RoutingMsg &routing_msg,
                                 const Injector &injector,
                                 std::vector<Point3d> &curvature_line) {
    cos_trans_theta_ = injector.cos_trans_theta;
    sin_trans_theta_ = injector.sin_trans_theta;

    stop_line_dist_eastimate_ = injector.stop_line_dist_eastimate;
    traffic_light_dist_ = injector.traffic_light_dist;

    has_stable_slope_ = injector.has_stable_slope;
    left_road_width_estimate_ = injector.left_road_width_estimate;
    lateral_road_width_ = injector.traffic_light_dist -
                          traffic_light_to_junction_dist -
                          stop_line_dist_eastimate_;

    turn_ = routing_msg.turn;
    routing_msg_ = routing_msg;
    if (turn_ == TURN::LEFT || turn_ == TURN::RIGHT) {
        RoadWidthEstimateByVehicle(vehicle_state, obstacles_msg);
        CalulateCurvatureLine(lane_info, pnc_lanes, vehicle_state,
                              curvature_line);
    } else if (turn_ == TURN::STRAIGHT) {
        CalulateStraightLine(lane_info, pnc_lanes, vehicle_state,
                             curvature_line);
    }

    return true;
}

void InterJunctionState::RoadWidthEstimateByVehicle(
    const PosFromIns &vehicle_state, const ObstaclesInfo &obstacles_msg) {
    double self_car_yaw = vehicle_state.yaw;

    bool left_to_right = false;
    bool right_to_left = false;
    double left_to_right_max_dist = 0;
    double right_to_left_min_dist = 1000;

    int32_t right_to_left_cnt = 0;
    double right_dist_sum = 0;
    double cos_car_yaw = std::cos(self_car_yaw);
    double sin_car_yaw = std::sin(self_car_yaw);

    for (const auto &obs : obstacles_msg.obstacles) {
        if (obs.type != ObsType::VEHICLE) {
            continue;
        }
        if (obs.length < 3.0 && obs.width < 1.2) {
            continue;
        }
        double vy = obs.state.linear_velocity.x * cos_car_yaw +
                    obs.state.linear_velocity.y * sin_car_yaw;
        double vx = obs.state.linear_velocity.x * sin_car_yaw -
                    obs.state.linear_velocity.y * cos_car_yaw;
        // vy = obs.state.linear_velocity.y;
        // vx = obs.state.linear_velocity.x;
        // vx = std::fabs(vx);

        // if (std::fabs(vy) > 2.0) {
        //     continue;
        // }
        double dx = obs.state.position.x - vehicle_state.position.x;
        double dy = obs.state.position.y - vehicle_state.position.y;

        double longitudinal_dist = dx * cos_car_yaw + dy * sin_car_yaw;
        double lateral_dist = dx * sin_car_yaw - dy * cos_car_yaw;

        double yaw =
            NormalizeAngle(obs.state.angular_deg.z - self_car_yaw + M_PI_2);
        // SG_INFO("id = %s,vx = %lf,vy = %lf,x = %lf,y = %lf",
        //         obs.track_id.c_str(), vx, vy, longitudinal_dist,
        //         lateral_dist);
        if (longitudinal_dist < stop_line_dist_eastimate_) {
            continue;
        }

        if (longitudinal_dist > traffic_light_dist_ + 10) {
            continue;
        }

        if (turn_ == TURN::LEFT) {
            // SG_INFO("select left");
            LeftObsPriorityInsert(longitudinal_dist - stop_line_dist_eastimate_,
                                  lateral_dist, vx, vy, yaw);
        } else if (turn_ == TURN::RIGHT) {
            RightObsPriorityInsert(
                longitudinal_dist - stop_line_dist_eastimate_, lateral_dist, vx,
                vy, yaw);
        }
    }
    if (turn_ == TURN::LEFT) {
        turn_dist_ = ObsLateralDistEastimate(left_obs_priority_set_);
    } else if (turn_ == TURN::RIGHT) {
        turn_dist_ = ObsLateralDistEastimate(right_obs_priority_set_);
    }
    turn_dist_ > 0 ? has_available_obstacle_ = true
                   : has_available_obstacle_ = false;
}
void InterJunctionState::LeftObsPriorityInsert(const double lon,
                                               const double lat,
                                               const double vx, const double vy,
                                               const double yaw) {
    // 1 select car from left to right
    int32_t lon_int = std::floor(lon);
    // SG_INFO("lon int = %d", lon_int);
    if (std::fabs(yaw) < max_tolerance_rad_) {
        if (lat < 0) {
            if (left_obs_priority_set_[priority_1].find(lon_int) ==
                left_obs_priority_set_[priority_1].end()) {
                left_obs_priority_set_[priority_1].insert(
                    std::make_pair(lon_int, 1));
                // SG_WARN("insert 1");
            } else {
                ++(left_obs_priority_set_[priority_1].find(lon_int)->second);
            }
        }
    }
    // 2 select car from right to left
    if (std::fabs(yaw) > M_PI - max_tolerance_rad_) {
        if (left_obs_priority_set_[priority_1].empty() == false) {
            if (lon_int < left_obs_priority_set_[priority_1].rbegin()->first) {
                return;
            }
        }
        if (lat < 0 && vx < -1.0) {
            if (left_obs_priority_set_[priority_0].find(lon_int) ==
                left_obs_priority_set_[priority_0].end()) {
                left_obs_priority_set_[priority_0].insert(
                    std::make_pair(lon_int, 1));
                // SG_WARN("insert 0");
            } else {
                ++(left_obs_priority_set_[priority_0].find(lon_int)->second);
            }
        }
        if (lat >= 0) {
            // lon_int +=
            //     static_cast<int32_t>(left_road_width_estimate_ *
            //     std::sin(yaw));
            if (left_obs_priority_set_[priority_2].find(lon_int) ==
                left_obs_priority_set_[priority_2].end()) {
                left_obs_priority_set_[priority_2].insert(
                    std::make_pair(lon_int, 1));
                // SG_WARN("insert 2");
            } else {
                ++(left_obs_priority_set_[priority_2].find(lon_int)->second);
            }
        }
    }
}

void InterJunctionState::RightObsPriorityInsert(const double lon,
                                                const double lat,
                                                const double vx,
                                                const double vy,
                                                const double yaw) {
    // 1 select car from right to left
    int32_t lon_int = std::floor(lon);
    // SG_INFO("lon int = %d", lon_int);
    if (std::fabs(yaw) > M_PI - max_tolerance_rad_) {
        if (lat > 0) {
            if (right_obs_priority_set_[priority_1].find(lon_int) ==
                right_obs_priority_set_[priority_1].end()) {
                right_obs_priority_set_[priority_1].insert(
                    std::make_pair(lon_int, 1));
            } else {
                ++(right_obs_priority_set_[priority_1].find(lon_int)->second);
            }
            SG_WARN("insert 1");
        }
    }

    // 2 select car from left to right
    if (std::fabs(yaw) < max_tolerance_rad_) {
        if (right_obs_priority_set_[priority_1].empty() == false) {
            if (lon_int > right_obs_priority_set_[priority_1].begin()->first) {
                return;
            }
        }
        if (lat > 0 && vx > 5.0) {
            if (right_obs_priority_set_[priority_0].find(lon_int) ==
                right_obs_priority_set_[priority_0].end()) {
                right_obs_priority_set_[priority_0].insert(
                    std::make_pair(lon_int, 1));

            } else {
                ++(right_obs_priority_set_[priority_0].find(lon_int)->second);
            }
            SG_WARN("insert 0");
        }
        if (lat <= 0 && std::fabs(yaw) < 0.3) {
            // lon_int +=
            //     static_cast<int32_t>(left_road_width_estimate_ *
            //     std::sin(yaw));
            if (right_obs_priority_set_[priority_2].find(lon_int) ==
                right_obs_priority_set_[priority_2].end()) {
                right_obs_priority_set_[priority_2].insert(
                    std::make_pair(lon_int, 1));
            }
            SG_WARN("insert 2");
        } else {
            ++(right_obs_priority_set_[priority_2].find(lon_int)->second);
            SG_WARN("insert 2");
        }
    }
}

double InterJunctionState::ObsLateralDistEastimate(
    const std::vector<std::map<int32_t, int32_t>> &obs_priority_set) {
    double return_val = -1;

    if (obs_priority_set[priority_0].empty() == false) {
        if (obs_priority_set[priority_0].size() == 1) {
            return_val = obs_priority_set[priority_0].begin()->first;
            // SG_INFO("return val 1 = %lf", return_val);
        } else {
            double arverage = 0.0;
            int32_t num = 0;
            for (const auto &val : obs_priority_set[priority_0]) {
                // SG_INFO("val 1= %d , 2= %d", val.first, val.second);
                arverage += val.first * val.second;
                num += val.second;
            }
            return_val = arverage / num;
            // SG_INFO("return val 2 = %lf", return_val);
        }
        return return_val;
    }
    double road_diff = 0.0;
    turn_ == TURN::LEFT ? road_diff = lane_width_standard * 2.5
                        : road_diff = -lane_width_standard * 2.5;
    if (obs_priority_set[priority_1].size() >= 2) {
        if (turn_ == TURN::LEFT) {
            return_val =
                obs_priority_set[priority_1].rbegin()->first + road_diff;
        } else if (turn_ == TURN::RIGHT) {
            return_val =
                obs_priority_set[priority_1].begin()->first + road_diff;
            // SG_INFO("first = %d,road_diff = %lf",
            //         obs_priority_set[priority_1].begin()->first, road_diff);
        }

        if (std::fabs(obs_priority_set[priority_1].rbegin()->first -
                          obs_priority_set[priority_1].begin()->first >
                      10)) {
            double arverage = 0.0;
            int32_t num = 0;
            for (const auto &val : obs_priority_set[priority_1]) {
                // SG_INFO("val 1= %d , 2= %d", val.first, val.second);
                arverage += val.first * val.second;
                num += val.second;
            }
            return_val = return_val = arverage / num + road_diff;
        }
        // SG_INFO("return val 3 = %lf", return_val);

        return return_val;
    }
    if (obs_priority_set[priority_2].empty() == false) {
        if (obs_priority_set[priority_2].size() == 1) {
            return_val = obs_priority_set[priority_2].begin()->first;
            // SG_INFO("return val 4 = %lf", return_val);
        } else {
            double arverage = 0.0;
            int32_t num = 0;
            for (const auto &val : obs_priority_set[priority_2]) {
                arverage += val.first * val.second;
                num += val.second;
                // SG_INFO("val 1= %d,2 = %d", val.first, val.second);
            }
            return_val = return_val = arverage / num;
            // SG_INFO("return val 5 = %lf", return_val);
        }
        return return_val;
    }
    // SG_INFO("return val 6 = %lf", return_val);
    return return_val;
}
void InterJunctionState::CalulateCurvatureLine(
    const LaneInfo &lane_info, const PnCLane &pnc_lanes,
    const PosFromIns &vehicle_state, std::vector<Point3d> &curvature_line) {
    double p1_x = 0;
    if (has_stable_slope_ == true) {
        // GetCenterLine(lane_info, vehicle_state, stop_line_dist_eastimate_ -
        // 6,
        //               curvature_line, p1_x);
        GetCenterLine(pnc_lanes, vehicle_state, stop_line_dist_eastimate_ - 6,
                      curvature_line, p1_x);
    } else {
        for (float y = 0; y < stop_line_dist_eastimate_ - 6; y += 1.0) {
            double enu_x = 0 * cos_trans_theta_ + y * sin_trans_theta_ +
                           vehicle_state.position.x;
            double enu_y = 0 * sin_trans_theta_ + y * cos_trans_theta_ +
                           vehicle_state.position.y;
            curvature_line.emplace_back(Point3d{enu_x, enu_y, 0});
        }
    }
    Point3d p1{p1_x, stop_line_dist_eastimate_ - 6, 0};
    Point3d p2{0, 0, 0};
    Point3d p3{0, 0, 0};

    NextRoadPointEstimateUseTraffic(p2, p3);
    NextRoadPointEstimateUseRouteMap(vehicle_state, p2, p3);

    // SG_INFO("p1.x = %lf,p1.y = %lf,p3 x = %lf,p3 y = %lf", p1.x, p1.y, p3.x,
    //         p3.y);
    BezierCurveEstimate(p1, p2, p3, vehicle_state, curvature_line);

    if (lateral_extended_line_.empty() == false) {
        lateral_extended_line_.clear();
    }

    double sign = 1;
    turn_ == TURN::LEFT ? sign = -1 : sign = 1;

    double sin_road_theta = std::sin(routing_msg_.next_road_theta);
    double cos_road_theta = std::cos(routing_msg_.next_road_theta);
    for (float s = 1; s < 150; s += 1.0) {
        double x = curvature_line.back().x + s * cos_road_theta;
        double y = curvature_line.back().y + s * sin_road_theta;
        lateral_extended_line_.emplace_back(Point3d{x, y, 0});
    }
}

void InterJunctionState::NextRoadPointEstimateUseTraffic(Point3d &p2,
                                                         Point3d &p3) {
    if (turn_ == TURN::LEFT) {
        p2 = Point3d{-left_road_width_estimate_, 0, 0};

        p3 = Point3d{p2.x, 0, 0};

        p3.y = lateral_road_width_ * 0.75 + stop_line_dist_eastimate_;

    } else if (turn_ == TURN::RIGHT) {
        p2 = Point3d{right_turn_lat, 0, 0};
        p3 = Point3d{right_turn_lat, right_turn_lon + stop_line_dist_eastimate_,
                     0};
    }
    // SG_INFO("traffic p3.y = %lf", p3.y);
}
void InterJunctionState::NextRoadPointEstimateUseRouteMap(
    const PosFromIns &vehicle_state, Point3d &p2, Point3d &p3) {
    if (routing_msg_.next_road_ref_points.size() < 2) {
        NextRoadPointEstimateUseTraffic(p2, p3);
        return;
    }
    if (turn_ == TURN::LEFT) {
        p2 = Point3d{-left_road_width_estimate_, 0, 0};
        p3 = Point3d{p2.x, 0, 0};
        // SG_INFO("turn left int3e");
    } else if (turn_ == TURN::RIGHT) {
        p2 = Point3d{right_turn_lat, 0, 0};
        p3 = Point3d{right_turn_lat, 0, 0};
        // SG_INFO("turn right int3e");
    }

    double self_car_yaw = vehicle_state.yaw;
    double cos_car_yaw = std::cos(self_car_yaw);
    double sin_car_yaw = std::sin(self_car_yaw);
    double dx =
        routing_msg_.next_road_ref_points[0].x - vehicle_state.position.x;
    double dy =
        routing_msg_.next_road_ref_points[0].y - vehicle_state.position.y;
    double y0 = dx * cos_car_yaw + dy * sin_car_yaw;
    double x0 = dx * sin_car_yaw - dy * cos_car_yaw;

    dx = routing_msg_.next_road_ref_points[1].x - vehicle_state.position.x;
    dy = routing_msg_.next_road_ref_points[1].y - vehicle_state.position.y;
    double y1 = dx * cos_car_yaw + dy * sin_car_yaw;
    double x1 = dx * sin_car_yaw - dy * cos_car_yaw;
    if (std::fabs(x1 - x0) < 1e-4) {
        NextRoadPointEstimateUseTraffic(p2, p3);
    } else {
        p3.y = y1 + (y1 - y0) / (x1 - x0) * (p3.x - x1);
    }
    // SG_INFO("route map p3.y = %lf", p3.y);
}

void InterJunctionState::NextRoadPointEstimateUseObstacle(Point3d &p2,
                                                          Point3d &p3) {
    if (turn_ == TURN::LEFT) {
        p2 = Point3d{-left_road_width_estimate_, 0, 0};
        p3 = Point3d{p2.x, turn_dist_ + stop_line_dist_eastimate_, 0};
    } else if (turn_ == TURN::RIGHT) {
        p2 = Point3d{right_turn_lat, 0, 0};
        p3 = Point3d{right_turn_lat, turn_dist_ + stop_line_dist_eastimate_, 0};
    }
    // SG_INFO("Obstacle p3.y = %lf", p3.y);
}

void InterJunctionState::CalulateStraightLine(
    const LaneInfo &lane_info, const PnCLane &pnc_lanes,
    const PosFromIns &vehicle_state, std::vector<Point3d> &curvature_line) {
    double p1_x = 0;

    double sin_road_theta = std::sin(routing_msg_.current_road_theta);
    double cos_road_theta = std::cos(routing_msg_.current_road_theta);

    if (has_stable_slope_ == true) {
        // GetCenterLine(lane_info, vehicle_state, lane_length_, curvature_line,
        //               p1_x);
        GetCenterLine(pnc_lanes, vehicle_state, lane_length_, curvature_line,
                      p1_x);
    } else {
        for (float s = 1; s < lane_length_; s += 1.0) {
            double x = vehicle_state.position.x + s * cos_road_theta;
            double y = vehicle_state.position.y + s * sin_road_theta;
            curvature_line.emplace_back(Point3d{x, y, 0});
        }
    }
    lateral_extended_line_.clear();
    // for (float s = 1; s < 150; s += 1.0) {
    //     double x = curvature_line.back().x + s * cos_road_theta;
    //     double y = curvature_line.back().y + s * sin_road_theta;
    //     lateral_extended_line_.emplace_back(Point3d{x, y, 0});
    // }
}

void InterJunctionState::BezierCurveEstimate(
    const Point3d &p1, const Point3d &p2, const Point3d &p3,
    const PosFromIns &vehicle_state, std::vector<Point3d> &curvature_line) {
    Point3d ctrl_p1{0, 0, 0};
    Point3d ctrl_p2{0, 0, 0};
    ctrl_p1.x = p1.x;
    ctrl_p2.y = p3.y;
    std::vector<double> param_a_kappa_min(4, 0);
    std::vector<double> param_b_kappa_min(4, 0);
    double kappa_min = 1e10;
    // SG_INFO("p3 x = %lf,p3 y = %lf", p3.x, p3.y);
    double delta_x = 1.0;
    turn_ == TURN::LEFT ? delta_x = -1.0 : delta_x = 1.0;
    double p3_x = std::fabs(p3.x);
    for (double dy = p3.y * 0.25; dy < p3.y * 0.75; dy += 1.0) {
        ctrl_p1.y = dy;
        for (double dx = p3_x * 0.25; dx < p3_x * 0.75; dx += 1.0) {
            ctrl_p2.x = dx * delta_x;

            std::vector<double> param_a(4, 0);
            param_a[0] = p1.x;
            param_a[1] = 3 * ctrl_p1.x - 3 * p1.x;
            param_a[2] = 3 * p1.x - 6 * ctrl_p1.x + 3 * ctrl_p2.x;
            param_a[3] = -p1.x + 3 * ctrl_p1.x - 3 * ctrl_p2.x + p3.x;

            std::vector<double> param_b(4, 0);
            param_b[0] = p1.y;
            param_b[1] = 3 * ctrl_p1.y - 3 * p1.y;
            param_b[2] = 3 * p1.y - 6 * ctrl_p1.y + 3 * ctrl_p2.y;
            param_b[3] = -p1.y + 3 * ctrl_p1.y - 3 * ctrl_p2.y + p3.y;
            double kappa = 0.0;
            // SG_INFO("dy = %lf,dx = %lf", dy, dx);
            for (float t = 0; t < 1.0; t += 0.1) {
                double x_dot =
                    param_a[1] + 2 * param_a[2] * t + 3 * param_a[3] * t * t;
                double x_ddot = 2 * param_a[2] + 6 * param_a[3] * t;
                double y_dot =
                    param_b[1] + 2 * param_b[2] * t + 3 * param_b[3] * t * t;
                double y_ddot = 2 * param_b[2] + 6 * param_b[3] * t;
                double x_dot_sqr_add_y_dot_sqr = x_dot * x_dot + y_dot * y_dot;
                if (std::fabs(x_dot_sqr_add_y_dot_sqr) < 1e-6) {
                    continue;
                }
                kappa += std::pow(x_dot * y_ddot - x_ddot * y_dot, 2) /
                         std::pow((x_dot_sqr_add_y_dot_sqr), 3);
            }
            if (kappa < kappa_min) {
                kappa_min = kappa;
                param_a_kappa_min = param_a;
                param_b_kappa_min = param_b;
            }
        }
    }
    int32_t cnt = 0;
    for (float t = 0.0; t < 1.0; t += 0.01) {
        double x = param_a_kappa_min[0] + param_a_kappa_min[1] * t +
                   param_a_kappa_min[2] * t * t +
                   param_a_kappa_min[3] * t * t * t;

        double y = param_b_kappa_min[0] + param_b_kappa_min[1] * t +
                   param_b_kappa_min[2] * t * t +
                   param_b_kappa_min[3] * t * t * t;

        double theta = (0.5 * M_PI - vehicle_state.yaw);

        double enu_x = x * cos_trans_theta_ + y * sin_trans_theta_ +
                       vehicle_state.position.x;
        double enu_y = -x * sin_trans_theta_ + y * cos_trans_theta_ +
                       vehicle_state.position.y;
        curvature_line.emplace_back(Point3d{enu_x, enu_y, 0});
    }
}

void InterJunctionState::GetCenterLine(const PnCLane &pnc_lanes,
                                       const PosFromIns &vehicle_state,
                                       const double length,
                                       std::vector<Point3d> &curvature_line,
                                       double &last_x) {
    const int32_t cnt = pnc_lanes.self_car_lane_cnt;

    if (std::fabs(pnc_lanes.lanes[cnt].fit_param[0]) < 1.0) {
        for (float l = 0; l < length; l += 1.0) {
            float s = 0;
            for (int32_t j = 0; j < pnc_lanes.lanes[cnt].fit_param.size();
                 ++j) {
                s += pnc_lanes.lanes[cnt].fit_param[j] * std::pow(l, j);
            }
            // SG_INFO("s = %lf,l = %lf", s, l);

            double enu_x = s * cos_trans_theta_ + l * sin_trans_theta_ +
                           vehicle_state.position.x;
            double enu_y = -s * sin_trans_theta_ + l * cos_trans_theta_ +
                           vehicle_state.position.y;

            Point3d point;
            point.x = enu_x;
            point.y = enu_y;
            point.z = 0.0;
            curvature_line.emplace_back(point);
        }
        last_x = 0.0;
        for (int32_t j = 0; j < pnc_lanes.lanes[cnt].fit_param.size(); ++j) {
            last_x += pnc_lanes.lanes[cnt].fit_param[j] * std::pow(length, j);
        }
    } else {
        float x = 0;
        for (float y = 0; y <= length; y += 1.0) {
            double enu_x = x * cos_trans_theta_ + y * sin_trans_theta_ +
                           vehicle_state.position.x;
            double enu_y = -x * sin_trans_theta_ + y * cos_trans_theta_ +
                           vehicle_state.position.y;
            curvature_line.emplace_back(Point3d{enu_x, enu_y, 0});
        }
    }
}

void InterJunctionState::GetCenterLine(const LaneInfo &lane_info,
                                       const PosFromIns &vehicle_state,
                                       const double length,
                                       std::vector<Point3d> &curvature_line,
                                       double &last_x) {
    if (lane_info.center_line.empty() == false) {
        int32_t count = 0;
        float delta_x = std::fabs(
            lane_info.center_line.front().points.front().position_m.x);
        for (int32_t i = 1; i < lane_info.center_line.size(); ++i) {
            const double &i_x =
                lane_info.center_line[i].points.front().position_m.x;
            if (delta_x > std::fabs(i_x)) {
                delta_x = std::fabs(i_x);
                count = i;
            }
        }
        last_x = lane_info.center_line[count].points.front().position_m.x;
        if (length < 0) {
            return;
        }
        for (int32_t i = 0; i < lane_info.center_line[count].points.size();
             ++i) {
            double x = lane_info.center_line[count].points[i].position_m.x;
            double y = lane_info.center_line[count].points[i].position_m.y;

            double enu_x = x * cos_trans_theta_ + y * sin_trans_theta_ +
                           vehicle_state.position.x;
            double enu_y = -x * sin_trans_theta_ + y * cos_trans_theta_ +
                           vehicle_state.position.y;

            if (length > 0 && y > length) {
                break;
            }
            curvature_line.emplace_back(Point3d{enu_x, enu_y, 0});
        }

        double last_y = lane_info.center_line[count].points.back().position_m.y;
        if (last_y < length) {
            double x = lane_info.center_line[count].points.back().position_m.x;
            for (float y = last_y; y <= length; y += 1.0) {
                double enu_x = x * cos_trans_theta_ + y * sin_trans_theta_ +
                               vehicle_state.position.x;
                double enu_y = -x * sin_trans_theta_ + y * cos_trans_theta_ +
                               vehicle_state.position.y;
                curvature_line.emplace_back(Point3d{enu_x, enu_y, 0});
            }
        }
    } else {
        float x = 0;
        for (float y = 0; y <= length; y += 1.0) {
            double enu_x = x * cos_trans_theta_ + y * sin_trans_theta_ +
                           vehicle_state.position.x;
            double enu_y = -x * sin_trans_theta_ + y * cos_trans_theta_ +
                           vehicle_state.position.y;
            curvature_line.emplace_back(Point3d{enu_x, enu_y, 0});
        }
    }
}
void InterJunctionState::Reset(void) {
    for (int32_t i = 0; i < left_obs_priority_set_.size(); ++i) {
        left_obs_priority_set_[i].clear();
    }
    for (int32_t i = 0; i < right_obs_priority_set_.size(); ++i) {
        right_obs_priority_set_[i].clear();
    }
    has_available_obstacle_ = false;
    left_road_width_estimate_ = 0.0;
    lateral_road_width_ = 0.0;

    right_turn_lat_dist_ = 0.0;
    right_turn_lon_dist_ = 0.0;
}
}  // namespace planning_lib
}  // namespace jarvis