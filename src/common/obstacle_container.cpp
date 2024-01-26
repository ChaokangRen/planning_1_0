#include "obstacle_container.h"

namespace jarvis {
namespace planning_lib {

ObstaclesInfo ObstacleContainer::Veh2Enu(const ObstaclesInfo &obs_info) {
    ObstaclesInfo obs_info_veh = obs_info;
    double ego_x = obs_info.location.position.x;
    double ego_y = obs_info.location.position.y;
    double ego_yaw = obs_info.location.yaw * M_PI / 180;

    ego_yaw += M_PI / 2;
    while (ego_yaw < 0) {
        ego_yaw += 2 * M_PI;
    }
    while (ego_yaw > 2 * M_PI) {
        ego_yaw -= 2 * M_PI;
    }
    for (int i = 0; i < obs_info.obstacles.size(); ++i) {
        double obs_x = obs_info.obstacles[i].state.position.x;
        double obs_y = obs_info.obstacles[i].state.position.y;
        double obs_yaw = obs_info.obstacles[i].state.angular_deg.z * M_PI / 180;
        while (obs_yaw < 0) {
            obs_yaw += 2 * M_PI;
        }
        while (obs_yaw > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }
        double obs_velx = obs_info.obstacles[i].state.linear_velocity.x;
        double obs_vely = obs_info.obstacles[i].state.linear_velocity.y;

        VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, obs_x, obs_y, obs_yaw,
                          obs_velx, obs_vely);

        obs_info_veh.obstacles[i].state.position.x = obs_x;
        obs_info_veh.obstacles[i].state.position.y = obs_y;
        obs_info_veh.obstacles[i].state.angular_deg.z = obs_yaw;
        // SG_INFO("%s-----obs_enu_yaw=%lf", obs_id.c_str(), obs_yaw);
        obs_info_veh.obstacles[i].state.linear_velocity.x = obs_velx;
        obs_info_veh.obstacles[i].state.linear_velocity.y = obs_vely;
        // SG_INFO("obs_yaw = %lf", obs_yaw);
    }

    return obs_info_veh;
}

void ObstacleContainer::VehCoord2EnuCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y, double &yaw, double &velx,
                                          double &vely) {
    double x_veh = x;
    double y_veh = y;
    double yaw_veh = yaw;
    double velx_veh = velx;
    double vely_veh = vely;
    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = (M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_veh * std::cos(rotation) + y_veh * std::sin(rotation) + ego_x;
    y = y_veh * std::cos(rotation) - x_veh * std::sin(rotation) + ego_y;
    yaw = yaw_veh + ego_yaw - M_PI / 2;
    while (yaw < 0) {
        yaw += 2 * M_PI;
    }
    while (yaw > 2 * M_PI) {
        yaw -= 2 * M_PI;
    }
    velx = velx_veh * std::cos(rotation) + vely_veh * std::sin(rotation);
    vely = vely_veh * std::cos(rotation) - velx_veh * std::sin(rotation);
    // //  SG_INFO("rotation = %lf,enu_x=%lf,enu_y=%lf", rotation, x, y);
}

void ObstacleContainer::VehCoord2EnuCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y) {
    // //  SG_INFO("ego_x = %lf,ego_y =%lf,ego_yaw=%lf,x=%lf,y=%lf", ego_x,
    // ego_y,
    //         ego_yaw, x, y);
    double x_veh = x;
    double y_veh = y;

    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = (M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_veh * std::cos(rotation) + y_veh * std::sin(rotation) + ego_x;
    y = y_veh * std::cos(rotation) - x_veh * std::sin(rotation) + ego_y;
}

void ObstacleContainer::EnuCoord2VehCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y, double &yaw, double &velx,
                                          double &vely) {
    double x_enu = x - ego_x;
    double y_enu = y - ego_y;
    double yaw_enu = yaw;
    double velx_enu = velx;
    double vely_enu = vely;
    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = -(M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_enu * std::cos(rotation) + y_enu * std::sin(rotation);
    y = y_enu * std::cos(rotation) - x_enu * std::sin(rotation);
    yaw = yaw_enu - ego_yaw + M_PI / 2;
    while (yaw < 0) {
        yaw += 2 * M_PI;
    }
    while (yaw > 2 * M_PI) {
        yaw -= 2 * M_PI;
    }
    velx = velx_enu * std::cos(rotation) + vely_enu * std::sin(rotation);
    vely = vely_enu * std::cos(rotation) - velx_enu * std::sin(rotation);
}

void ObstacleContainer::EnuCoord2VehCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y) {
    double x_enu = x - ego_x;
    double y_enu = y - ego_y;

    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = -(M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_enu * std::cos(rotation) + y_enu * std::sin(rotation);
    y = y_enu * std::cos(rotation) - x_enu * std::sin(rotation);
}

ObstaclesInfo ObstacleContainer::GetObsEnu() {
    return obstacle_info_enu_;
}

}  // namespace planning_lib
}  // namespace jarvis