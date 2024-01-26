#pragma once

#include <cmath>

#include "common.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {

class ObstacleContainer {
public:
    ObstacleContainer(const ObstaclesInfo &obstacle_info)
        : obstacle_info_veh_(obstacle_info) {
        obstacle_info_enu_ = Veh2Enu(obstacle_info);
    }

    ObstaclesInfo GetObsEnu();

private:
    ObstaclesInfo obstacle_info_veh_;
    ObstaclesInfo obstacle_info_enu_;

    ObstaclesInfo Veh2Enu(const ObstaclesInfo &obs_info);

    void VehCoord2EnuCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y,
                           double &yaw, double &velx, double &vely);
    void VehCoord2EnuCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y);
    void EnuCoord2VehCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y,
                           double &yaw, double &velx, double &vely);
    void EnuCoord2VehCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y);
};
}  // namespace planning_lib
}  // namespace jarvis