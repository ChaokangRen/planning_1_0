#include "obstacle.h"

#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {

double Obstacle::Speed() const {
    return speed_;
}
const std::string &Obstacle::Id() const {
    return obstacle_msg_.track_id;
}
const Polygon2d &Obstacle::GetObstaclePolygon() const {
    return obstacle_polygon_;
}
const Box2d &Obstacle::GetObstacleBox() const {
    return obstacle_box_;
}
const ObsTrajectory &Obstacle::Trajectorys() const {
    return obstacle_msg_.obs_trajectories[0];
}

Box2d Obstacle::GetBoundingBox(const PathPoint &point) const {
    double length_buff = 0.0;
    PathPoint point_buff = point;
    double width_debuff = 0.0;
    double tail_buff = 1.0;
    if (obstacle_msg_.type == ObsType::VEHICLE) {
        tail_buff = 1.5;
        point_buff.position_m.x +=
            -tail_buff * std::sin(M_PI_2 - point.theta_rad);
        point_buff.position_m.y +=
            -tail_buff * std::cos(M_PI_2 - point.theta_rad);
        length_buff += tail_buff;
    } else if (obstacle_msg_.type == ObsType::PEDESTRIAN) {
        length_buff += 1.3;
        width_debuff += 1.3;
    } else if (obstacle_msg_.type == ObsType::CYCLIST) {
        length_buff += 0.5;
        width_debuff += 0.5;
    }
    if (is_virtual_ == true && is_static_ == true) {
        length_buff = 0.0;
        width_debuff = 0.0;
    }
    return Box2d(Vec2d(point_buff.position_m.x, point_buff.position_m.y),
                 point.theta_rad, obstacle_msg_.length + length_buff,
                 obstacle_msg_.width + width_debuff);
}

bool Obstacle::GetPointAtTime(const double t, PathPoint *path_point) const {
    if (obstacle_msg_.obs_trajectories.empty()) {
        return false;
    }
    const ObsTrajectory &trajectory = obstacle_msg_.obs_trajectories[0];

    if (t < trajectory.points.front().relative_time_s ||
        t > trajectory.points.back().relative_time_s) {
        return false;
    }
    for (int32_t i = 1; i < trajectory.points.size(); ++i) {
        if (trajectory.points[i - 1].relative_time_s <= t &&
            trajectory.points[i].relative_time_s > t) {
            float slope = (t - trajectory.points[i - 1].relative_time_s) /
                          (trajectory.points[i].relative_time_s -
                           trajectory.points[i - 1].relative_time_s);

            path_point->position_m.x =
                trajectory.points[i - 1].path_point.position_m.x * (1 - slope) +
                trajectory.points[i].path_point.position_m.x * slope;

            path_point->position_m.y =
                trajectory.points[i - 1].path_point.position_m.y * (1 - slope) +
                trajectory.points[i].path_point.position_m.y * slope;

            path_point->theta_rad =
                trajectory.points[i - 1].path_point.theta_rad * (1 - slope) +
                trajectory.points[i].path_point.theta_rad * slope;

            path_point->kappa =
                trajectory.points[i - 1].path_point.kappa * (1 - slope) +
                trajectory.points[i].path_point.kappa * slope;

            path_point->dkappa =
                trajectory.points[i - 1].path_point.dkappa * (1 - slope) +
                trajectory.points[i].path_point.dkappa * slope;

            path_point->ddkappa = 0.0;

            break;
        }
    }
    return true;
}
}  // namespace planning_lib
}  // namespace jarvis