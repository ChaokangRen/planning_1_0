#pragma once

#include <Eigen/Eigen>

#include "common.h"
#include "planning_interface.h"
#include "runge_kutta.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {
class ObstaclePrediction {
public:
    ObstaclePrediction() = default;

    explicit ObstaclePrediction(const ObstaclesInfo &obs_info);

    ObstaclesInfo ObstacleInEnu() {
        return obses_;
    }

private:
    void VehCoord2EnuCoord(const ObstaclesInfo &obs_info);

    void SetTransMat(const double yaw, const double pitch, const double roll);

    void TrajectoryPrediction();

    void GenObsTrajectory(const ObstacleMsg &obs, ObsTrajectory *trajectory);

private:
    ObstaclesInfo obses_;
    RungeKutta runge_kutta_;

    Eigen::Matrix3d trans_mat_;
    Eigen::Vector3d pos_in_veh_;
    Eigen::Vector3d pos_in_enu_;
};
}  // namespace planning_lib
}  // namespace jarvis