#include "rtk_generator.h"

#include <jsoncpp/json/json.h>

#include <fstream>

#include "cartesian_frenet_conversion.h"
#include "sglog/sglog.h"
#include "sgtime/sgtime.h"
namespace jarvis {
namespace planning_lib {

// const double REF_VEL = 13.0;
const double REF_VEL = 14.5;

bool RtkGenerator::InitRtkTrajectory(const std::string &rtk_path) {
    Json::Reader reader;
    Json::Value root;

    std::string rtk_trajectory_path = rtk_path;
    SG_INFO("rtk path = %s", rtk_trajectory_path.c_str());

    std::vector<PathPoint> global_path;
    std::ifstream in(rtk_trajectory_path, std::ios::binary);

    if (!in.is_open()) {
        SG_ERROR("json file open failed!");
    }

    if (reader.parse(in, root)) {
        int32_t path_size = root["reference_line"].size();
        const auto &root_ref = root["reference_line"];
        for (int32_t i = 0; i < path_size; ++i) {
            const auto &point = root_ref[i];
            PathPoint lane_point;
            lane_point.position_m.x = point[0].asDouble();
            lane_point.position_m.y = point[1].asDouble();
            lane_point.position_m.z = 0;
            lane_point.theta_rad = point[2].asDouble();
            lane_point.dkappa = point[3].asDouble();
            if (lane_point.dkappa < 8.5) {
                if (i > 7332 && i < 92839) {
                    if (i > 41346 && i < 49911) {
                        lane_point.dkappa = 7.5;
                    } else if (i < 10081) {
                        lane_point.dkappa = 7.5;
                    } else {
                        lane_point.dkappa = 8.5;
                    }

                } else {
                    lane_point.dkappa = std::fmax(lane_point.dkappa, 6.5);
                }
            }
            // lane_point.dkappa = REF_VEL;
            // if (lane_point.dkappa > REF_VEL) {
            //     lane_point.dkappa = REF_VEL;
            // }

            global_path.emplace_back(lane_point);
        }
    }
    center_lines_.emplace_back(std::move(global_path));

    // WriteObsLine();

    return true;
}

bool RtkGenerator::InitRtkLeftTrajectory(const std::string &rtk_path) {
    Json::Reader reader;
    Json::Value root;

    std::string rtk_trajectory_path = rtk_path;
    rtk_trajectory_path =
        "/usr/share/jarvis/planning/resource/rtk_trajectory/"
        "qidi_city_demo_3_7.json";
    // "test.json";

    std::ifstream in(rtk_trajectory_path, std::ios::binary);

    std::vector<PathPoint> global_left_path;

    if (!in.is_open()) {
        SG_ERROR("json file open failed!");
    }

    if (reader.parse(in, root)) {
        int32_t path_size = root["reference_line"].size();
        const auto &root_ref = root["reference_line"];
        for (int32_t i = 0; i < path_size; ++i) {
            const auto &point = root_ref[i];
            PathPoint lane_point;
            lane_point.position_m.x = point[0].asDouble();
            lane_point.position_m.y = point[1].asDouble();
            lane_point.position_m.z = 0;
            lane_point.theta_rad = point[2].asDouble();
            lane_point.dkappa = point[3].asDouble();
            if (lane_point.dkappa < 8.5) {
                if (i > 7332 && i < 92839) {
                    if (i > 41346 && i < 49911) {
                        lane_point.dkappa = 7.5;
                    } else if (i < 10081) {
                        lane_point.dkappa = 7.5;
                    } else {
                        lane_point.dkappa = 8.5;
                    }

                } else {
                    lane_point.dkappa = std::fmax(lane_point.dkappa, 6.5);
                }
                lane_point.dkappa = REF_VEL;
            }
            global_left_path.emplace_back(lane_point);
        }
    }
    center_lines_.emplace_back(std::move(global_left_path));
    return true;
}
static bool first_search_rtk = true;
bool RtkGenerator::UpdateLocalPathForPlanning(const PosFromIns &pos_ins) {
    bool return_value = false;
    for (int i = 0; i < center_lines_.size(); ++i) {
        return_value = UpdateLocalPathForPlanning(pos_ins, i);
    }
    return return_value;
}

bool RtkGenerator::UpdateLocalPathForPlanning(const PosFromIns &pos_ins,
                                              const int32_t lane_id) {
    if (lane_id < 0 || lane_id >= center_lines_.size()) {
        return false;
    }
    count_ = count_vec_[lane_id];

    std::vector<PathPoint> &lane = center_lines_[lane_id];
    if (count_ >= lane.size()) {
        SG_WARN("reference line over");
        return false;
    }
    int32_t start = count_ > 100 ? count_ - 100 : 0;
    int32_t end = count_ + 1000 > lane.size() ? lane.size() : count_ + 1000;
    if (first_search_rtk == true) {
        first_search_rtk = false;
        start = 0;

        end = lane.size() - int32_t(lane.size() / 10);
        // end = lane.size();
    }
    double dist_min = std::numeric_limits<double>::max();
    int32_t count_min = -1;
    for (int32_t i = start; i < end; ++i) {
        double delta_x = lane[i].position_m.x - pos_ins.position.x;
        double delta_y = lane[i].position_m.y - pos_ins.position.y;

        double dist = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (dist_min > dist) {
            dist_min = dist;
            count_min = i;
        }
    }
    count_ = count_min;
    count_vec_[lane_id] = count_;

    double dist_len = 0;
    int32_t np = 1;
    std::vector<PathPoint> local_path;
    for (int32_t i = count_min + 1; i < lane.size(); ++i) {
        double dx = lane[i].position_m.x - lane[i - 1].position_m.x;
        double dy = lane[i].position_m.y - lane[i - 1].position_m.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        while (dist_len + dist > np * delta_s_) {
            double ref_x =
                lerp(lane[i - 1].position_m.x, dist_len, lane[i].position_m.x,
                     dist_len + dist, np * delta_s_);
            double ref_y =
                lerp(lane[i - 1].position_m.y, dist_len, lane[i].position_m.y,
                     dist_len + dist, np * delta_s_);
            double ref_theta =
                lerp(lane[i - 1].theta_rad, dist_len, lane[i].theta_rad,
                     dist_len + dist, np * delta_s_);

            PathPoint point;
            point.position_m.x = ref_x;
            point.position_m.y = ref_y;
            point.position_m.z = 0;
            point.theta_rad = ref_theta;
            point.s_m = np * delta_s_;
            point.dkappa = lane[i].dkappa;

            local_path.emplace_back(point);
            np++;
            if (np > local_points_num_) {
                break;
            }
        }
        dist_len += dist;
        if (np > local_points_num_) {
            break;
        }
    }

    if (np < local_points_num_) {
        return false;
    }

    local_path_[lane_id] = std::move(local_path);

    double final_x = 4203.38;
    double final_y = 2534.15;
    double final_yaw = 0.8126;

    if (pos_ins.velocity.x < 0.27) {
        double dx = final_x - pos_ins.position.x;
        double dy = final_y - pos_ins.position.y;
        // SG_INFO("vel = %lf,dx = %lf,dy = %lf", pos_ins.velocity.x, dx, dy);
        if (std::fabs(dx) < 100 && std::fabs(dy) < 100) {
            if (std::fabs(final_yaw - pos_ins.yaw) < 0.2) {
                double cos_car_yaw = std::cos(pos_ins.yaw);
                double sin_car_yaw = std::sin(pos_ins.yaw);

                double longitudinal_dist = dx * cos_car_yaw + dy * sin_car_yaw;
                // double lateral_dist = dx * sin_car_yaw - dy *
                // cos_car_yaw;

                // SG_INFO("longitudinal_dist = %lf", longitudinal_dist);
                if (longitudinal_dist < -0.1) {
                    for (int32_t k = 0; k < count_vec_.size(); ++k) {
                        count_vec_[k] = 0;
                    }
                }
            }
        }
    }

    return true;
}

void RtkGenerator::GetLaneBoundary(const int32_t lane_id, double *left_boundary,
                                   double *right_boundary) {
    double min_width = 1.5;
    if (lane_id == 1) {
        *left_boundary = min_width;
        *right_boundary = -min_width;
        return;
    }
    int32_t current_count = count_vec_[lane_id];
    if (current_count < 6462) {
        *left_boundary = 2.2;
        *right_boundary = -min_width;
    } else if (current_count >= 6462 && current_count < 24931) {
        *left_boundary = min_width;
        *right_boundary = -min_width;
    } else if (current_count >= 24931 && current_count < 28490) {
        *left_boundary = min_width;
        // *right_boundary = -4.5;
        *right_boundary = -min_width;
    } else if (current_count >= 28490 && current_count < 94290) {
        *left_boundary = min_width;
        *right_boundary = -min_width;
    } else if (current_count >= 94290 && current_count < 97200) {
        *left_boundary = 3.0;
        *right_boundary = -min_width;
    } else {
        *left_boundary = min_width;
        *right_boundary = -min_width;
    }
    return;
}

VirtualObs RtkGenerator::EndVirtualObs(const PosFromIns &pos_ins) {
    if (std::fabs(pos_ins.yaw - end_point_.z) > M_PI_4) {
        return VirtualObs();
    }
    double dx = pos_ins.position.x - end_point_.x;
    double dy = pos_ins.position.y - end_point_.y;

    if (dx * dx + dy * dy < 1600) {
        return VirtualObs(end_point_.x, end_point_.y, end_point_.z, 0.1, 5, 2.0,
                          pos_ins);
    }
    return VirtualObs();
}

}  // namespace planning_lib
}  // namespace jarvis