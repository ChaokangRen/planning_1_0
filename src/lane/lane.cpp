#include <jsoncpp/json/json.h>

#include "lane.h"

namespace jarvis {
namespace planning_lib {

void Lane::Init() {}
bool Lane::IsStopLineOnLocalPath(const LineSegment2d &stop_line,
                                 const std::vector<PathPoint> &local_path) {
    int32_t step_len = 15;

    for (int32_t i = 0; i < local_path.size() - step_len; i += step_len) {
        int32_t end_len = i + step_len;
        if (end_len > local_path.size() - 1) {
            end_len = local_path.size() - 1;
        }
        Vec2d p1(local_path[i].position_m.x, local_path[i].position_m.y);
        Vec2d p2(local_path[end_len].position_m.x,
                 local_path[end_len].position_m.y);

        LineSegment2d path_line(p1, p2);

        if (stop_line.HasIntersect(path_line) == true) {
            return true;
        }
    }
    return false;
}

void Lane::Process(const LaneInfo &lane_info, const SpeedFromVeh &speed_veh,
                   const PosFromIns &pos_ins_,
                   const std::vector<PathPoint> &local_path) {
    UpdateHasStopLine(lane_info, local_path);
    double trans_theta = (2.5 * M_PI - pos_ins_.yaw);
    sin_trans_theta_ = std::sin(trans_theta);
    cos_trans_theta_ = std::cos(trans_theta);
    veh_x_ = pos_ins_.position.x;
    veh_y_ = pos_ins_.position.y;

    if (stop_line_status_ == StopLineStatus::None) {
        double dx = lane_info.location.position.x - vehicle_pos_pre.x();
        double dy = lane_info.location.position.y - vehicle_pos_pre.y();
        // SG_INFO("dx = %lf,dy = %lf", dx, dy);
        // SG_INFO("dx = %lf,dy = %lf", dx, dy);
        if (dx * dx + dy * dy > 900) {
            if (has_stop_line_ == true) {
                stop_line_status_ = StopLineStatus::Front;
            }
        }
        is_actiove_move_ = false;
        // SG_INFO("StopLineStatus::None");
    } else if (stop_line_status_ == StopLineStatus::Front) {
        UpdateVehicleMoveDistance(lane_info);
        StopLineProcess(lane_info, local_path);
        Variance();
        is_actiove_move_ = true;
        if (has_stop_line_ == false) {
            stop_line_status_ = StopLineStatus::Rear;
        }
        if (current_veh2stopline_dist_ < -0.5) {
            Reset();
            stop_line_status_ = StopLineStatus::None;
        }
        // SG_INFO("StopLineStatus::Front");
    } else if (stop_line_status_ == StopLineStatus::Rear) {
        UpdateVehicleMoveDistance(lane_info);
        if (current_veh2stopline_dist_ < -0.5) {
            Reset();
            stop_line_status_ = StopLineStatus::None;
        }
        if (has_stop_line_ == true) {
            stop_line_status_ = StopLineStatus::Front;
        }
        if (current_veh2stopline_dist_ > 100 && has_stop_line_ == false) {
            stop_line_status_ = StopLineStatus::None;
        }
        // SG_INFO("StopLineStatus::Rear");
    }
    // SG_INFO("current dist = %lf,normal dist = %lf,Variance = %lf",
    //         current_veh2stopline_dist_, distance_to_stop_line_, variance_);
}  // namespace planning_lib

void Lane::StopLineProcess(const LaneInfo &lane_info,
                           const std::vector<PathPoint> &local_path) {
    double current_stop_line_dist_min = -1;
    if (has_stop_line_ == true) {
        std::vector<LaneLineMsg const *> stop_line_vec;
        for (const LaneLineMsg &lane_line_msg : lane_info.lane_lines_msg) {
            if (lane_line_msg.type == LaneLineMsg::Type::STOP_LINE) {
                Vec2d p1(lane_line_msg.lane_line.front().point.position_m.x,
                         lane_line_msg.lane_line.front().point.position_m.y);
                Vec2d p2(lane_line_msg.lane_line.back().point.position_m.x,
                         lane_line_msg.lane_line.back().point.position_m.y);

                double enu_p1_x = p1.x() * cos_trans_theta_ +
                                  p1.y() * sin_trans_theta_ + veh_x_;
                double enu_p1_y = -p1.x() * sin_trans_theta_ +
                                  p1.y() * cos_trans_theta_ + veh_y_;

                double enu_p2_x = p2.x() * cos_trans_theta_ +
                                  p2.y() * sin_trans_theta_ + veh_x_;
                double enu_p2_y = -p2.x() * sin_trans_theta_ +
                                  p2.y() * cos_trans_theta_ + veh_y_;

                Vec2d enu_p1(enu_p1_x, enu_p1_y);
                Vec2d enu_p2(enu_p2_x, enu_p2_y);
                if (IsStopLineOnLocalPath(LineSegment2d(enu_p1, enu_p2),
                                          local_path) == true) {
                    stop_line_vec.emplace_back(&lane_line_msg);
                }
            }
        }
        if (stop_line_vec.empty() == false && has_stop_line_ == true) {
            current_stop_line_dist_min =
                stop_line_vec.front()->lane_line.front().point.position_m.y;

            for (int32_t i = 1; i < stop_line_vec.size(); ++i) {
                if (current_stop_line_dist_min >
                    stop_line_vec[i]->lane_line.front().point.position_m.y) {
                    current_stop_line_dist_min =
                        stop_line_vec[i]->lane_line.front().point.position_m.y;
                }
            }
        }
        double error = current_stop_line_dist_min - current_veh2stopline_dist_;
        // SG_INFO("error = %lf,dist1 = %lf,dist2 = %lf",
        //         current_stop_line_dist_min, current_veh2stopline_dist_);

        if (error > 10 && current_veh2stopline_dist_ < 15 && variance_ < 10.0 &&
            variance_ > 0) {
        } else {
            if (current_stop_line_dist_min > 0 && error < 30) {
                StopLineFilter(current_stop_line_dist_min, lane_info);
            }
        }
    }
}

void Lane::UpdateHasStopLine(const LaneInfo &lane_info,
                             const std::vector<PathPoint> &local_path) {
    bool current_has_stop_line = false;
    if (has_stop_line_ == false) {
        hasnot_stop_line_count_ = 0;
        for (const LaneLineMsg &lane_line_msg : lane_info.lane_lines_msg) {
            if (lane_line_msg.type == LaneLineMsg::Type::STOP_LINE) {
                if (pre_has_stop_line_ == true) {
                    ++stop_line_count_;
                }
                current_has_stop_line = true;
            }
        }

        pre_has_stop_line_ = current_has_stop_line;

        if (stop_line_count_ >= 5) {
            has_stop_line_ = true;
        }
    } else if (has_stop_line_ == true) {
        stop_line_count_ = 0;
        for (const LaneLineMsg &lane_line_msg : lane_info.lane_lines_msg) {
            if (lane_line_msg.type == LaneLineMsg::Type::STOP_LINE) {
                current_has_stop_line = true;
                break;
            }
            current_has_stop_line = false;
        }
        if (pre_has_stop_line_ == false && current_has_stop_line == false) {
            ++hasnot_stop_line_count_;
        }
        if (hasnot_stop_line_count_ > 5) {
            has_stop_line_ = false;
        }
        pre_has_stop_line_ = current_has_stop_line;
    }
}

void Lane::StopLineFilter(const double current_dist,
                          const LaneInfo &lane_info) {
    if (variance_ > 0 && variance_ < 0.1) {
        if (current_dist > current_veh2stopline_dist_) {
            return;
        }
    }
    distance_to_stop_line_ =
        UpdateStopLineList(vehicle_move_distance_ + current_dist);
    // SG_INFO("distance_to_stop_line_ = %lf,current_dist = %lf",
    //         distance_to_stop_line_, current_dist);
}

double Lane::UpdateStopLineList(const double stop_line_dist) {
    stop_line_list_.push_back(stop_line_dist);
    if (stop_line_list_.size() > list_max_cnt_) {
        stop_line_list_.pop_front();
    }
    if (stop_line_list_.size() <= 2) {
        return 999;
    }
    double forgetting_factor = 1.0;
    double sum = 0.0;
    double sum_factor = 0.0;
    double factor = 1.0;
    for (const double val : stop_line_list_) {
        sum += val * factor;
        sum_factor += factor;
        factor = factor * forgetting_factor;
    }
    return sum / sum_factor;
}
void Lane::UpdateVehicleMoveDistance(const LaneInfo &lane_info) {
    if (is_actiove_move_ == true) {
        double dx = lane_info.location.position.x - vehicle_pos_pre.x();
        double dy = lane_info.location.position.y - vehicle_pos_pre.y();
        vehicle_move_distance_ += std::sqrt(dx * dx + dy * dy);
    }
    vehicle_pos_pre.set_x(lane_info.location.position.x);
    vehicle_pos_pre.set_y(lane_info.location.position.y);
    current_veh2stopline_dist_ =
        distance_to_stop_line_ - vehicle_move_distance_;
    if (current_veh2stopline_dist_ < -0.9 &&
        stop_line_status_ == StopLineStatus::Front) {
        current_veh2stopline_dist_ = 999;
    }
    // SG_INFO("current_veh2stopline_dist_ = %lf", current_veh2stopline_dist_);
}

void Lane::Reset(void) {
    current_veh2stopline_dist_ = 999;
    vehicle_move_distance_ = 0.0;
    observe_stop_line_dist_ = 0;
    has_stop_line_ = false;
    has_stop_line_pre_ = false;
    variance_ = -1;
    distance_to_stop_line_ = 999;
    stop_line_list_.clear();
    // SG_INFO("reset");
}

void Lane::Variance(void) {
    if (stop_line_list_.size() < list_max_cnt_ - 1) {
        variance_ = -1;
        return;
    }
    double sum = 0;
    for (const double val : stop_line_list_) {
        sum += val;
    }
    double arv = sum / stop_line_list_.size();
    sum = 0.0;
    for (const double val : stop_line_list_) {
        sum += (val - arv) * (val - arv);
    }
    variance_ = sum / stop_line_list_.size();
}
}  // namespace planning_lib
}  // namespace jarvis