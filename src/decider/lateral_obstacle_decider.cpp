#include "lateral_obstacle_decider.h"

namespace jarvis {
namespace planning_lib {
void LateralObstacleDecider::Process(
    const Lane &stop_lane, const ReferenceLine &reference_lines,
    const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
    const PnCLane &pnc_center_lines, const DriveState &drive_state,
    const SLPoint &adc_point, std::vector<Obstacle> &obstacles) {
    double adc_left_l = adc_point.l + half_vehicle_width_;
    double adc_right_l = adc_point.l - half_vehicle_width_;
    double adc_l = adc_point.l;

    bool has_left_lane = false;
    bool has_right_lane = false;

    for (const FitCurve &line : pnc_center_lines.lanes) {
        if (line.digit_id == -1) {
            has_left_lane = true;
        } else if (line.digit_id == 1) {
            has_right_lane = true;
        }
    }

    double x_min_lf = std::numeric_limits<double>::max();
    double x_max_lr = -std::numeric_limits<double>::max();
    double x_min_rf = std::numeric_limits<double>::max();
    double x_max_rr = -std::numeric_limits<double>::max();
    int sub_lf = -1;
    int sub_lr = -1;
    int sub_rf = -1;
    int sub_rr = -1;

    double obs_x = 0.0;
    double obs_y = 0.0;

    for (int i = 0; i < obstacles.size(); ++i) {
        obs_x = obstacles[i].GetPosInVeh().x();
        obs_y = obstacles[i].GetPosInVeh().y();
        if (has_left_lane && obs_y > 0 && obs_y < 5) {
            if (obs_x < 40 && obs_x > 0 && obs_x < x_min_lf) {
                x_min_lf = obs_x;
                sub_lf = i;
            }
            if (obs_x > -40 && obs_x < 0 && obs_x > x_max_lr) {
                x_max_lr = obs_x;
                sub_lr = i;
            }
        }
        if (has_right_lane && obs_y < 0 && obs_y > -5) {
            if (obs_x < 40 && obs_x > 0 && obs_x < x_min_rf) {
                x_min_rf = obs_x;
                sub_rf = i;
            }
            if (obs_x > -40 && obs_x < 0 && obs_x > x_max_rr) {
                x_max_rr = obs_x;
                sub_rr = i;
            }
        }
    }
    if (sub_lf != -1) {
        obstacles[sub_lf].SetPositionType(Obstacle::POSITIONTYPE::FRONTLEFT);
    }
    if (sub_lr != -1) {
        obstacles[sub_lr].SetPositionType(Obstacle::POSITIONTYPE::REARLEFT);
    }
    if (sub_rf != -1) {
        obstacles[sub_rf].SetPositionType(Obstacle::POSITIONTYPE::FRONTRIGHT);
    }
    if (sub_rr != -1) {
        obstacles[sub_rr].SetPositionType(Obstacle::POSITIONTYPE::REARRIGHT);
    }

    for (Obstacle &obstacle : obstacles) {
        SG_INFO("%s --- obs_x = %lf,obs_y = %lf", obstacle.Id().c_str(),
                obstacle.GetPosInVeh().x(), obstacle.GetPosInVeh().y());

        // check obstacle is static
        if (obstacle.IsVirtual() == true) {
            continue;
        }
        if (obstacle.IsStatic() == false) {
            double adc_obs_yaw_error =
                NormalizeAngle(obstacle.Heading() - pos_ins.yaw);
            SG_INFO("%s is dynamic yaw = %lf", obstacle.Id().c_str(),
                    adc_obs_yaw_error);
            if (adc_obs_yaw_error < opposite_veh_yaw_) {
                continue;
            }
        } else {
            SG_INFO("%s is static yaw = %lf", obstacle.Id().c_str());
        }

        if (obstacle.GetPosInVeh().x() < -10) {
            continue;
        }

        double obs_right_l = obstacle.GetObsSlBoundary().start_l;
        double obs_left_l = obstacle.GetObsSlBoundary().end_l;

        double left_l = adc_right_l - obs_left_l;
        double right_l = obs_right_l - adc_left_l;

        SG_INFO("obs_left_l = %lf,obs_right_l = %lf,left_l = %lf,right_l = %lf",
                obs_left_l, obs_right_l, left_l, right_l);

        double buff_dist =
            obstacle.IsStatic() ? safe_buff_dist_ : lateral_safe_dist_;

        if (left_l > buff_dist || right_l > buff_dist) {
            obstacle.SetBehavior(Obstacle::BEHAVIORTYPE::IGNORE);
            continue;
        }

        double right_nudge_l = obs_right_l - adc_l;
        double left_nudge_l = adc_l - obs_left_l;

        if (left_nudge_l > safe_buff_dist_) {
            obstacle.SetBehavior(Obstacle::BEHAVIORTYPE::LEFTNUDGE);
            continue;
        }

        if (right_nudge_l > safe_buff_dist_) {
            obstacle.SetBehavior(Obstacle::BEHAVIORTYPE::RIGHTNUDGE);
            continue;
        }
        obstacle.SetBehavior(Obstacle::BEHAVIORTYPE::FOLLOW);
        obstacle.SetPositionType(Obstacle::POSITIONTYPE::FRONT);
    }

    for (Obstacle &obstacle : obstacles) {
        SG_INFO("%s behavior is %d", obstacle.Id().c_str(),
                obstacle.Behavior());
    }
}
}  // namespace planning_lib
}  // namespace jarvis