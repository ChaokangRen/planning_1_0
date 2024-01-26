#include "obstacle_sim.h"

#include <jsoncpp/json/json.h>

#include <cmath>
#include <fstream>

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {
void ObstacleGenerator::InitObstacleTrajectory() {
    Json::Reader reader;
    Json::Value root;

    std::string rtk_trajectory_path =
        "/usr/share/jarvis/planning/resource/rtk_trajectory/obstacle_line.json";

    std::ifstream in(rtk_trajectory_path, std::ios::binary);

    if (!in.is_open()) {
        SG_ERROR("json file open failed!");
    }

    if (reader.parse(in, root)) {
        int32_t path_size = root["obstacle_line"].size();
        const auto &root_ref = root["obstacle_line"];
        for (int32_t i = 0; i < path_size; ++i) {
            const auto &point = root_ref[i];
            PathPoint lane_point;
            lane_point.position_m.x = point[0].asDouble();
            lane_point.position_m.y = point[1].asDouble();
            lane_point.position_m.z = 0;
            lane_point.theta_rad = point[2].asDouble();
            obstacle1_path.emplace_back(lane_point);
        }
    }

    return;
}

bool ObstacleGenerator::UpdateObstacle(const PosFromIns &pos_ins,
                                       ObstacleMsg *obstacle) {
    static int32_t count = 100;
    static int32_t step = 7;
    static bool start_flag = false;
    obstacle->state.position.x = obstacle1_path[count].position_m.x;
    obstacle->state.position.y = obstacle1_path[count].position_m.y;
    obstacle->state.position.z = 2;
    obstacle->state.angular_deg.x = 0;
    obstacle->state.angular_deg.y = 0;
    obstacle->state.angular_deg.z = obstacle1_path[count].theta_rad;
    obstacle->state.linear_velocity.x = 3;
    obstacle->state.linear_velocity.y = 3;
    obstacle->state.linear_velocity.z = 3;

    obstacle->width = 2;
    obstacle->height = 2;
    obstacle->length = 5;
    obstacle->is_static = false;
    obstacle->type = ObsType::VEHICLE;
    obstacle->track_count = 2;
    obstacle->track_id = "dynamic obs";

    double dist = std::sqrt(
        (pos_ins.position.x - obstacle1_path[count].position_m.x) *
            (pos_ins.position.x - obstacle1_path[count].position_m.x) +
        (pos_ins.position.y - obstacle1_path[count].position_m.y) *
            (pos_ins.position.y - obstacle1_path[count].position_m.y));

    if (dist > 18 && start_flag == false) {
        return false;
    } else {
        start_flag = true;
    }

    if (start_flag == true) {
        float t = 0.1;
        ObsTrajectory trajectory;
        for (int i = 0; i < 120; i++) {
            TrajectoryPoint point;
            point.path_point.position_m.x =
                obstacle1_path[count + i * step].position_m.x;
            point.path_point.position_m.y =
                obstacle1_path[count + i * step].position_m.y;
            point.path_point.theta_rad =
                obstacle1_path[count + i * step].theta_rad;
            point.relative_time_s = i * t;
            point.path_point.s_m = i * step * 0.1;
            point.path_point.kappa = 0;
            point.path_point.dkappa = 0;
            trajectory.points.emplace_back(point);
        }
        obstacle->obs_trajectories.emplace_back(trajectory);
        count = count + step;
    }
    if (count >= 872) {
        step = 0;
    }

    return true;
}

}  // namespace planning_lib
}  // namespace jarvis