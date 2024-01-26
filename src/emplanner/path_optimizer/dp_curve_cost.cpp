#include "dp_curve_cost.h"

#include <cmath>

#include "sglog/sglog.h"
#include "sgtime/sgtime.h"
namespace jarvis {
namespace planning_lib {

bool DpCurveCost::Init(const PathOptimizerConf &path_optimizer_conf,
                       const VehicleParams &vehicle_params) {
    eval_time_interval_ = path_optimizer_conf.eval_time_interval;
    dp_path_cost_sample_dist_ = path_optimizer_conf.dp_path_cost_sample_dist;
    dp_path_cost_ = path_optimizer_conf.dp_path_cost;
    dp_path_dl_cost_ = path_optimizer_conf.dp_path_dl_cost;
    dp_path_ddl_cost_ = path_optimizer_conf.dp_path_ddl_cost;
    dp_static_obs_cost_ = path_optimizer_conf.dp_static_obs_cost;
    dp_dynamic_obs_cost_small_ = path_optimizer_conf.dp_dynamic_obs_cost_small;
    dp_dynamic_obs_cost_big_ = path_optimizer_conf.dp_dynamic_obs_cost_big;
    has_collision_cost_ = path_optimizer_conf.has_collision_cost;
    safety_distance_ = path_optimizer_conf.safety_distance;
    obstacle_collision_distance_ =
        path_optimizer_conf.obstacle_collision_distance;
    risk_obstacle_collision_distance_ =
        path_optimizer_conf.risk_obstacle_collision_distance;
    obstacle_ignore_distance_ = path_optimizer_conf.obstacle_ignore_distance;
    static_obs_safe_ratio_ = path_optimizer_conf.static_obs_safe_ratio;

    vehicle_length_ = vehicle_params.vehicle_length;
    vehicle_width_ = vehicle_params.vehicle_width;
    front_edge_to_center_ = vehicle_params.front_edge_to_center;
    back_edge_to_center_ = vehicle_params.back_edge_to_center;
    left_edge_to_center_ = vehicle_params.left_edge_to_center;
    right_edge_to_center_ = vehicle_params.right_dege_to_center;
    rearview_mirror_width_ = vehicle_params.rear_view_mirror_width;
    return true;
}

bool DpCurveCost::Construct(const SpeedData &heuristic_speed_data,
                            const ReferenceLine *reference_line,
                            const std::vector<Obstacle> &obstacles,
                            const SLPoint init_sl_point) {
    dynamic_obstacle_boxes_.clear();
    reference_line_ = reference_line;
    init_sl_point_ = init_sl_point;
    heuristic_speed_data_ = heuristic_speed_data;
    obstacles_ = &obstacles;
    double total_time = std::min(heuristic_speed_data_.TotalTime(), 12.0);
    num_of_time_stamps_ =
        static_cast<uint32_t>(total_time / eval_time_interval_);

    for (const Obstacle &ptr_path_obstacle : obstacles) {
        if ((&ptr_path_obstacle)->IsStatic()) {
            continue;
        }

        std::vector<Box2d> box_by_time;
        for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {
            PathPoint obs_point_by_time;
            if (!ptr_path_obstacle.GetPointAtTime(t * eval_time_interval_,
                                                  &obs_point_by_time)) {
                continue;
            }

            Box2d obstacle_box =
                Box2d(Vec2d(obs_point_by_time.position_m.x,
                            obs_point_by_time.position_m.y),
                      obs_point_by_time.theta_rad, ptr_path_obstacle.Length(),
                      ptr_path_obstacle.Width());

            box_by_time.push_back(obstacle_box);
        }
        dynamic_obstacle_boxes_.emplace_back(std::move(box_by_time));
    }
    return true;
}

DpCost DpCurveCost::CalculateCurveCost(const QuinticPolynomialCurve &curve,
                                       const float start_s,
                                       const float end_s) const {
    DpCost sum_cost = 0;

    sum_cost += CalculateDpPathCost(curve, start_s, end_s);

    sum_cost += CalculateDpStaticObsCost(curve, start_s, end_s);

    sum_cost += CalculateDpDynamicObsCost(curve, start_s, end_s);
    return sum_cost;
}

DpCost DpCurveCost::CalculateDpPathCost(const QuinticPolynomialCurve &curve,
                                        const float start_s,
                                        const float end_s) const {
    DpCost dp_path_cost = 0;
    for (float curve_s = 0; curve_s < (end_s - start_s);
         curve_s += dp_path_cost_sample_dist_) {
        double curve_l = curve.CalcuteCurveValue(curve_s);
        double iter_s = start_s + curve_s;
        std::vector<double> lane_line_l;
        for (int i = 0; i < reference_line_->CenterLines().size(); ++i) {
            lane_line_l.emplace_back(
                reference_line_->CenterLines()[i].GetLaneLineBoundary(iter_s));
        }
        double delta_l = 1e10;
        for (int i = 0; i < lane_line_l.size(); ++i) {
            double dist_l = std::fabs(lane_line_l[i] - curve_l);
            if (dist_l < delta_l) {
                delta_l = dist_l;
            }
        }

        // dp_path_cost += curve_l * curve_l * dp_path_cost_;
        dp_path_cost += delta_l * delta_l * dp_path_cost_;

        double curve_dl = curve.CalcuteDerivativeCurveValue(curve_s);
        dp_path_cost += curve_dl * curve_dl * dp_path_dl_cost_;

        double curve_ddl = curve.CalcuteSecDerivativeCurveValue(curve_s);
        dp_path_cost += curve_ddl * curve_ddl * dp_path_ddl_cost_;

        double curve_dddl = curve.CalcuteThdDerivativeCurveValue(curve_s);
        dp_path_cost += curve_dddl * curve_dddl * dp_path_ddl_cost_;
    }

    return dp_path_cost;
}

DpCost DpCurveCost::CalculateDpStaticObsCost(
    const QuinticPolynomialCurve &curve, const float start_s,
    const float end_s) const {
    DpCost dp_static_cost = 0;
    for (float curve_s = 0; curve_s < (end_s - start_s);
         curve_s += dp_path_cost_sample_dist_) {
        double curve_l = curve.CalcuteCurveValue(curve_s);
        for (int32_t i = 0; i < obstacles_->size(); ++i) {
            if ((*obstacles_)[i].IsStatic() == true) {
                dp_static_cost += CalcuteSingelStaticObsCost(
                    (*obstacles_)[i].GetObsSlBoundary(), curve_s + start_s,
                    curve_l);
            }
        }
    }
    return dp_static_cost;
}

Box2d DpCurveCost::GetBoxFromSLPoint(const SLPoint &sl_point,
                                     const float dl) const {
    Vec2d xy_point;
    reference_line_->SLToXY(sl_point, &xy_point);

    float one_minus_kappa_r_d =
        1 - reference_line_->GetCurrentSKappa(sl_point.s) * sl_point.l;
    float delta_theta = std::atan2(dl, one_minus_kappa_r_d);
    float theta = NormalizeAngle(
        delta_theta + reference_line_->GetCurrentSHeading(sl_point.s));

    return Box2d(xy_point, theta, vehicle_length_, vehicle_width_);
}

DpCost DpCurveCost::CalculateDpDynamicObsCost(
    const QuinticPolynomialCurve &curve, const float start_s,
    const float end_s) const {
    DpCost dp_dynamic_cost = 0;

    SpeedPoint vehicle_init_speed;

    heuristic_speed_data_.EvaluateByTime(0, &vehicle_init_speed);

    float time_stamp =
        (start_s - init_sl_point_.s) / vehicle_init_speed.velocity;
    for (int32_t index = static_cast<int32_t>(time_stamp / eval_time_interval_);
         index < num_of_time_stamps_;
         ++index, time_stamp += eval_time_interval_) {
        SpeedPoint speed_point;

        heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);

        float ref_s = speed_point.s + init_sl_point_.s;
        if (ref_s < start_s) {
            continue;
        }
        if (ref_s > end_s) {
            break;
        }
        const float s = ref_s - start_s;

        const float l = curve.CalcuteCurveValue(s);

        const float dl = curve.CalcuteDerivativeCurveValue(s);

        Box2d ego_box = GetBoxFromSLPoint(SLPoint{ref_s, l}, dl);

        for (const std::vector<Box2d> &obstacle_trajectory :
             dynamic_obstacle_boxes_) {
            int32_t obs_index = index;
            if (obstacle_trajectory.empty() == true) {
                continue;
            }
            if (obs_index >= obstacle_trajectory.size()) {
                obs_index = obstacle_trajectory.size() - 1;
            }

            if (ego_box.DistToBoxByRoundCheck(obstacle_trajectory[obs_index]) >
                obstacle_ignore_distance_) {
                continue;
            }

            Polygon2d obstacle_box(obstacle_trajectory[obs_index]);

            double single_cost =
                CalcuteCostBetweenObsPolygon(Polygon2d(ego_box), obstacle_box);

            dp_dynamic_cost += single_cost;
        }
    }

    return dp_dynamic_cost;
}

DpCost DpCurveCost::CalcuteSingelStaticObsCost(
    const SLStaticBoundary &static_obs_tmp, const double curve_s,
    const double curve_l) const {
    DpCost obstacle_cost = 0;
    SLStaticBoundary static_obs = static_obs_tmp;

    static_obs.start_l -= 1;
    static_obs.end_l += 0.5;

    float adc_front_s = curve_s + front_edge_to_center_;
    float adc_rear_s = curve_s - back_edge_to_center_;
    float adc_left_l = curve_l + left_edge_to_center_ + rearview_mirror_width_;
    float adc_right_l =
        curve_l - right_edge_to_center_ - rearview_mirror_width_;

    bool is_overlap = !(
        (adc_front_s < static_obs.start_s || adc_rear_s > static_obs.end_s) ||
        (adc_left_l < static_obs.start_l) || (adc_right_l > static_obs.end_l));
    if (is_overlap) {
        return obstacle_cost += has_collision_cost_;
    }
    if (adc_rear_s > static_obs.end_s + 10) {
        return obstacle_cost;
    }

    float delta_l = std::fmax(adc_right_l - static_obs.end_l,
                              static_obs.start_l - adc_left_l);

    if (static_obs.start_s - adc_front_s > 30) {
        return obstacle_cost;
    }

    obstacle_cost +=
        dp_static_obs_cost_ * Sigmoid(obstacle_collision_distance_ - delta_l);
    return obstacle_cost;
}

DpCost DpCurveCost::CalcuteCostBetweenObsPolygon(
    const Polygon2d &ego_poly, const Polygon2d &obs_poly) const {
    DpCost obs_cost = 0;

    double distance = ego_poly.DistanceTo(obs_poly);

    if (distance > obstacle_ignore_distance_) {
        return obs_cost;
    }
    obs_cost += dp_dynamic_obs_cost_big_ *
                Sigmoid(obstacle_collision_distance_ - distance);

    obs_cost += dp_dynamic_obs_cost_small_ *
                Sigmoid(risk_obstacle_collision_distance_ - distance);
    return obs_cost;
}
}  // namespace planning_lib
}  // namespace jarvis