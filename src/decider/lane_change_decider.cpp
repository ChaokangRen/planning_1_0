#include "lane_change_decider.h"

namespace jarvis {
namespace planning_lib {

Vec2d LaneChangeDecider::Obs2Ego(const PosFromIns &pos_ins, double obs_px,
                                 double obs_py) {
    Vec2d point_sl;
    double ego_x = pos_ins.position.x;
    double ego_y = pos_ins.position.y;

    // SG_INFO("yaw=%f", pos_ins.yaw);
    //  double ego_heading = NormalizeAngle(pos_ins.yaw);
    double ego_heading = -(M_PI / 2 - pos_ins.yaw);
    // SG_INFO("yaw1=%f", ego_heading);

    double obs_x = obs_px;
    double obs_y = obs_py;

    // SG_INFO("ego_x1=%f,ego_y=%f,obs_x=%f,obs_y=%f", ego_x, ego_y, obs_x,
    // obs_y);
    //  double dist = sqrt((ego_x - obs_x) * (ego_x - obs_x) +
    //                     (ego_y - obs_y) * (ego_y - obs_y));
    //  SG_INFO("center_dis = %f", dist);

    Vec2d obs2ego(obs_x - ego_x, obs_y - ego_y);
    // point_sl = obs2ego.Rotate(ego_heading);
    point_sl =
        Vec2d(obs2ego.x() * cos(ego_heading) + obs2ego.y() * sin(ego_heading),
              -obs2ego.x() * sin(ego_heading) + obs2ego.y() * cos(ego_heading));
    // s:x,l:y
    return point_sl;
}

Vec2d LaneChangeDecider::Obs2Ego(const PosFromIns &pos_ins, PathPoint endp) {
    Vec2d point_sl;
    double ego_x = pos_ins.position.x;
    double ego_y = pos_ins.position.y;

    // SG_INFO("yaw=%f", pos_ins.yaw);
    //  double ego_heading = NormalizeAngle(pos_ins.yaw);
    double ego_heading = -(M_PI / 2 - pos_ins.yaw);
    // SG_INFO("yaw1=%f", ego_heading);

    double obs_x = endp.position_m.x;
    double obs_y = endp.position_m.y;

    // SG_INFO("ego_x=%f,ego_y=%f,obs_x=%f,obs_y=%f", ego_x, ego_y, obs_x,
    // obs_y);
    //  double dist = sqrt((ego_x - obs_x) * (ego_x - obs_x) +
    //                     (ego_y - obs_y) * (ego_y - obs_y));
    //  SG_INFO("center_dis = %f", dist);

    Vec2d obs2ego(obs_x - ego_x, obs_y - ego_y);
    // point_sl = obs2ego.Rotate(ego_heading);
    point_sl =
        Vec2d(obs2ego.x() * cos(ego_heading) + obs2ego.y() * sin(ego_heading),
              -obs2ego.x() * sin(ego_heading) + obs2ego.y() * cos(ego_heading));
    // s:x,l:y
    return point_sl;
}

void LaneChangeDecider::Process(const PnCLane &pnc_center_lines,
                                const std::vector<Obstacle> &obstacles,
                                const PosFromIns &pos_ins,
                                const SpeedFromVeh &speed_veh,
                                const ReferenceLine &refline,
                                std::vector<PathPoint> &path_points) {
    // SG_INFO("lane change decider is processing");
    PathPoint end_path_point_enu = path_points[path_points.size() - 1];
    end_path_point_ = Obs2Ego(pos_ins, end_path_point_enu);
    // SG_INFO("endp_l=%f,endp_s=%f", end_path_point_.x(), end_path_point_.y());
    vel_exp_ = kVelExpDefaultValue;
    if (!IsLaneChange(pnc_center_lines, refline)) {
        SG_INFO("not chang lane!!!");
        return;
    }

    IsClearToChangeLine(obstacles, pos_ins, speed_veh, refline, path_points);
    return;
}

bool LaneChangeDecider::IsLaneChange(const PnCLane &pnc_center_lines,
                                     const ReferenceLine &refline) {
    double min_ls = 999;
    double min_le = 999;
    double index_ls = 0;
    double index_le = 0;

    if (std::abs(end_path_point_.x()) > kCrossingTurn) {
        return false;
    }

    for (int i = 0; i < pnc_center_lines.lanes.size(); i++) {
        // SG_INFO("center_line_l=%f",
        //         lane_info.center_line[i].points.front().position_m.x);
        double centerl = pnc_center_lines.lanes[i].fit_param[0];
        double centerl2endpoint =
            pnc_center_lines.lanes[i].fit_param[1] * end_path_point_.y() +
            pnc_center_lines.lanes[i].fit_param[0];
        if (min_ls > std::abs(centerl)) {
            min_ls = std::abs(centerl);
            index_ls = i;
        }
        double end2center = std::abs(end_path_point_.x() - centerl2endpoint);
        if (min_le > end2center) {
            min_le = end2center;
            index_le = i;
        }
    }
    // SG_INFO("index_ls=%f,index_le=%f", index_ls, index_le);
    if (index_le == index_ls) {
        return false;
    }
    double l1 =
        pnc_center_lines.lanes[index_le].fit_param[1] * kFitParamDefaultS +
        pnc_center_lines.lanes[index_le].fit_param[0];
    double l2 =
        pnc_center_lines.lanes[index_ls].fit_param[1] * kFitParamDefaultS +
        pnc_center_lines.lanes[index_ls].fit_param[0];
    lane_change_point_.l = (l1 + l2) / 2;
    return true;
}

void LaneChangeDecider::CalLcPoint(const PosFromIns &pos_ins,
                                   std::vector<PathPoint> &path_points) {
    std::vector<Vec2d> path_point_ego_vec;
    for (int i = 0; i < path_points.size() - 1; i++) {
        Vec2d tmp_sl1 = Obs2Ego(pos_ins, path_points[i]);
        Vec2d tmp_sl2 = Obs2Ego(pos_ins, path_points[i + 1]);
        // SG_INFO("tmp_sl1l=%f,tmp_sl1s=%f", tmp_sl1.x(), tmp_sl1.y());
        double deltl1 = tmp_sl1.x() - lane_change_point_.l;
        double deltl2 = tmp_sl2.x() - lane_change_point_.l;
        if (deltl1 * deltl2 < 0) {
            lane_change_point_.s = tmp_sl1.y();
        }
    }
}

void LaneChangeDecider::IsClearToChangeLine(
    const std::vector<Obstacle> obstacles, const PosFromIns &pos_ins,
    const SpeedFromVeh &speed_veh, const ReferenceLine &ref,
    std::vector<PathPoint> &path_points) {
    double ego_length = 4.8;
    double ego_width = 1.8;
    Box2d ego_box(Vec2d(pos_ins.position.x, pos_ins.position.y), pos_ins.yaw,
                  ego_length, ego_width);

    double ego_start_s = -ego_length / 2;
    double ego_end_s = ego_length / 2;
    double ego_start_l = -ego_width / 2;
    double ego_end_l = ego_width / 2;

    double ego_center_l = 0;
    double ego_center_s = 0;

    for (const auto &obs : obstacles) {
        // if (obs.IsStatic()) {
        //     SG_INFO("static_obs_id =%s", obs.Id().c_str());
        //     continue;
        // }
        if (obs.Id() == "virtual wall obstacle") {
            continue;
        }
        // SG_INFO("obs_id =%s", obs.Id().c_str());
        double obs_start_s = std::numeric_limits<double>::max();
        double obs_end_s = -std::numeric_limits<double>::max();
        double obs_start_l = std::numeric_limits<double>::max();
        double obs_end_l = -std::numeric_limits<double>::max();

        for (const auto &obs_point : obs.GetObstacleBox().GetAllCorners()) {
            // SG_INFO("obs_x =%f,obs_y =%f", obs_point.y(), obs_point.x());
            double pos_s;
            double pos_l;
            Vec2d tmp_sl = Obs2Ego(pos_ins, obs_point.x(), obs_point.y());
            pos_s = tmp_sl.y();
            pos_l = tmp_sl.x();
            obs_start_s = std::fmin(obs_start_s, pos_s);
            obs_end_s = std::fmax(obs_end_s, pos_s);
            obs_start_l = std::fmin(obs_start_l, pos_l);
            obs_end_l = std::fmax(obs_end_l, pos_l);
        }
        // SG_INFO("obs_starts =%f,obs_ends=%f,obs_startl=%f,obs_endl=%f",
        //         obs_start_s, obs_end_s, obs_start_l, obs_end_l);
        double obs_near_l =
            std::fmin(std::fabs(obs_end_l), std::fabs(obs_start_l));
        if (obs_near_l == std::fabs(obs_end_l)) {
            obs_near_l = obs_end_l;
        } else {
            obs_near_l = obs_start_l;
        }

        if (std::abs(obs_near_l) > kLaneChangeConcerningLateralArea) {
            continue;
        }
        if (obs_start_s > ego_end_s + kForwardMinSafeDistanceOnSameDirection) {
            // SG_INFO("obs_start_s > ego_end_s");
            continue;
        }

        double ego_heading = NormalizeAngle(pos_ins.yaw);
        double obs_heading = NormalizeAngle(obs.Heading());

        bool same_direction = true;
        double heading_difference =
            NormalizeAngle(std::abs(ego_heading - obs_heading));
        same_direction = heading_difference < M_PI_2;

        double kForwardSafeDistance = 0.0;
        double kBackwardSafeDistance = 0.0;
        if (same_direction) {
            kForwardSafeDistance = std::fmax(
                kForwardMinSafeDistanceOnSameDirection,
                (speed_veh.speed_mps - obs.Speed()) * kSafeTimeOnSameDirection);
            kBackwardSafeDistance = std::fmax(
                kBackwardMinSafeDistanceOnSameDirection,
                (obs.Speed() - speed_veh.speed_mps) * kSafeTimeOnSameDirection);
        } else {
            kForwardSafeDistance =
                std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                          (speed_veh.speed_mps + obs.Speed()) *
                              kSafeTimeOnOppositeDirection);
            kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
        }

        if ((HysteresisFilter(ego_start_s - obs_end_s, kBackwardSafeDistance,
                              kDistanceBuffer) &&
             HysteresisFilter(obs_start_s - ego_end_s, kForwardSafeDistance,
                              kDistanceBuffer))) {
            CalLcPoint(pos_ins, path_points);
            // SG_WARN("lc_point_l=%f,s=%f", lane_change_point_.l,
            //         lane_change_point_.s);
            double dis = ego_start_s - obs_end_s;
            double dis1 = lane_change_point_.s;
            double dis2 = lane_change_point_.s -
                          kLaneChangeDefaultTime * speed_veh.speed_mps;
            vel_exp_ = std::min(vel_exp_, (dis2 * obs.Speed() / (dis + dis1)));
            if (vel_exp_ < 0) {
                vel_exp_ = 0;
            }
        }
    }
}

bool LaneChangeDecider::HysteresisFilter(const double obstacle_distance,
                                         const double safe_distance,
                                         const double distance_buffer) {
    SG_INFO("actual_dis = %f,safe_dis = %f", obstacle_distance,
            safe_distance + distance_buffer);
    return obstacle_distance < safe_distance + distance_buffer;
}

}  // namespace planning_lib
}  // namespace jarvis