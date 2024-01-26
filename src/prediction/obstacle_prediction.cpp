#include "obstacle_prediction.h"

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

ObstaclePrediction::ObstaclePrediction(const ObstaclesInfo &obs_info)
    : obses_(obs_info) {
    VehCoord2EnuCoord(obs_info);
}

const double angle_to_rad = M_PI / 180;
void ObstaclePrediction::VehCoord2EnuCoord(const ObstaclesInfo &obs_info) {
    double yaw = -(2 * M_PI - obs_info.vehicle_pos.rotation.z * angle_to_rad);
    double pitch = obs_info.vehicle_pos.rotation.y;
    double roll = obs_info.vehicle_pos.rotation.x;

    SetTransMat(yaw, pitch, roll);

    DebugInfo debug_info;
    std::string obs_str = "{obstacles:";

    for (int i = 0; i < obs_info.obstacles.size(); ++i) {
        pos_in_veh_(0) = obs_info.obstacles[i].position.center.x;
        pos_in_veh_(1) = obs_info.obstacles[i].position.center.y;
        pos_in_veh_(2) = obs_info.obstacles[i].position.center.z;

        pos_in_enu_ = trans_mat_ * pos_in_veh_;
        obses_.obstacles[i].position.center.x =
            pos_in_enu_(0) + obses_.vehicle_pos.pos.x;
        obses_.obstacles[i].position.center.y =
            pos_in_enu_(1) + obses_.vehicle_pos.pos.y;
        obses_.obstacles[i].position.center.z =
            pos_in_enu_(2) + obses_.vehicle_pos.pos.z;

        obses_.obstacles[i].position.rotation.z =
            obs_info.vehicle_pos.rotation.z * angle_to_rad + M_PI_2 +
            obs_info.obstacles[i].position.rotation.z * angle_to_rad;

        double speed =
            std::sqrt(obs_info.obstacles[i].position.linear_velocity.x *
                          obs_info.obstacles[i].position.linear_velocity.x +
                      obs_info.obstacles[i].position.linear_velocity.y *
                          obs_info.obstacles[i].position.linear_velocity.y);
        obses_.obstacles[i].position.linear_velocity.x = speed;

        if (speed < 0.5) {
            obses_.obstacles[i].is_static = true;
        } else {
            obses_.obstacles[i].is_static = false;
        }
        // obses_.obstacles[i].is_static = false;
        GenObsTrajectory(obses_.obstacles[i],
                         &(obses_.obstacles[i].obs_trajectory));

        // add debug info msg
        std::string obs_str =
            "(x:" + std::to_string(obses_.obstacles[i].position.center.x) + ",";
        obs_str +=
            "y:" + std::to_string(obses_.obstacles[i].position.center.y) + ",";
        obs_str +=
            "yaw:" + std::to_string(obses_.obstacles[i].position.rotation.z) +
            ",";
        obs_str += "length:" + std::to_string(obses_.obstacles[i].length) + ",";
        obs_str += "width:" + std::to_string(obses_.obstacles[i].width);
        obs_str += "),";

        obs_str += "obstacles_end}";
        debug_info.SetDebugInfo(obs_str);
    }
}

void ObstaclePrediction::SetTransMat(const double yaw, const double pitch,
                                     const double roll) {
    double sin_yaw = std::sin(yaw);
    double cos_yaw = std::cos(yaw);
    double sin_pitch = std::sin(pitch);
    double cos_pitch = std::cos(pitch);
    double sin_roll = std::sin(roll);
    double cos_roll = std::cos(roll);

    // trans_mat_(0, 0) = cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll;
    // trans_mat_(0, 1) = sin_yaw * cos_pitch;
    // trans_mat_(0, 2) = cos_yaw * sin_pitch - sin_yaw * sin_pitch * cos_roll;

    // trans_mat_(1, 0) = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    // trans_mat_(1, 1) = cos_yaw * cos_pitch;
    // trans_mat_(1, 2) = -cos_yaw * sin_pitch * cos_roll - sin_pitch *
    // sin_roll;

    // trans_mat_(2, 0) = -cos_pitch * sin_roll;
    // trans_mat_(2, 1) = sin_pitch;
    // trans_mat_(2, 2) = cos_pitch * cos_roll;

    trans_mat_(0, 0) = cos_yaw;
    trans_mat_(0, 1) = -sin_yaw;
    trans_mat_(0, 2) = 0;

    trans_mat_(1, 0) = sin_yaw;
    trans_mat_(1, 1) = cos_yaw;
    trans_mat_(1, 2) = 0;

    trans_mat_(2, 0) = 0;
    trans_mat_(2, 1) = 0;
    trans_mat_(2, 2) = 1;
}

void ObstaclePrediction::GenObsTrajectory(const ObstacleMsg &obs,
                                          ObsTrajectory *trajectory) {
    if (trajectory->size() != 0) {
        trajectory->clear();
    }

    double speed = std::sqrt(
        obs.position.linear_velocity.x * obs.position.linear_velocity.x +
        obs.position.linear_velocity.y * obs.position.linear_velocity.y);
    std::vector<double> kesi{speed,
                             0,
                             obs.position.rotation.z,
                             0,
                             obs.position.center.y,
                             obs.position.center.x};
    double time_gap = 0.1;
    double prediction_time = 5;
    double delta_f = 0.0;
    double acc = 0;
    for (float t = 0.1; t < prediction_time; t += time_gap) {
        kesi = runge_kutta_.Solver(kesi, delta_f, acc, time_gap);
        ObsTrajectoryPoint point;
        point.x = kesi[5];
        point.y = kesi[4];
        point.theta = kesi[2];
        point.kappa = 0;
        point.dkappa = 0;
        point.relative_time = t;
        point.s = obs.position.linear_velocity.x * t;
        trajectory->emplace_back(point);
    }
}
}  // namespace planning_lib
}  // namespace jarvis