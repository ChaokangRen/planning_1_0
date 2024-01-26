#include "virtual_obs.h"

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

VirtualObs::VirtualObs(const double x, const double y, const double yaw,
                       const double length, const double width,
                       const double height, const PosFromIns &pos_ins) {
    ObstacleMsg virtual_obs_msg;
    virtual_obs_msg.state.position.x = x;
    virtual_obs_msg.state.position.y = y;
    virtual_obs_msg.state.position.z = virtual_height_ / 2;

    virtual_obs_msg.state.angular_deg.x = 0;
    virtual_obs_msg.state.angular_deg.y = 0;
    virtual_obs_msg.state.angular_deg.z = yaw;

    virtual_obs_msg.state.linear_velocity.x = 0;
    virtual_obs_msg.state.linear_velocity.y = 0;
    virtual_obs_msg.state.linear_velocity.z = 0;

    virtual_obs_msg.width = width;
    virtual_obs_msg.length = length;
    virtual_obs_msg.height = height;
    virtual_obs_msg.is_static = false;
    virtual_obs_msg.type = ObsType::UNKNOWN_UNMOVABLE;

    virtual_obs_msg.track_count = 1;
    virtual_obs_msg.track_id = "virtual wall obstacle";

    double time_interval = 0.1;
    ObsTrajectory trajectory;
    for (double t = 0.0; t <= total_time_; t += time_interval) {
        TrajectoryPoint point;
        point.path_point.position_m.x = virtual_obs_msg.state.position.x;
        point.path_point.position_m.y = virtual_obs_msg.state.position.y;
        point.path_point.theta_rad = virtual_obs_msg.state.angular_deg.z;
        point.relative_time_s = t;
        point.path_point.s_m = 0.0;
        point.path_point.kappa = 0.0;
        point.path_point.dkappa = 0.0;
        trajectory.points.emplace_back(point);
    }
    virtual_obs_msg.obs_trajectories.emplace_back(trajectory);

    obstacle_ = Obstacle(virtual_obs_msg);
    obstacle_.SetVirtual(true);
    obstacle_.SetStatic(false);
    virtual_wall_.height = height;
    virtual_wall_.width = width;
    virtual_wall_.thickness = length;

    double trans_x = virtual_obs_msg.state.position.x - pos_ins.position.x;
    double trans_y = virtual_obs_msg.state.position.y - pos_ins.position.y;
    double veh_yaw = pos_ins.yaw - M_PI_2;
    double sin_yaw = std::sin(veh_yaw);
    double cos_yaw = std::cos(veh_yaw);

    virtual_wall_.center_point_m.x = trans_x * cos_yaw + trans_y * sin_yaw;
    virtual_wall_.center_point_m.y = -trans_x * sin_yaw + trans_y * cos_yaw;
    virtual_wall_.center_point_m.z = virtual_height_ / 2;
    virtual_wall_.theta_rad =
        -(M_PI_2 - virtual_obs_msg.state.angular_deg.z) - veh_yaw;
    is_actived_ = true;
}
}  // namespace planning_lib
}  // namespace jarvis