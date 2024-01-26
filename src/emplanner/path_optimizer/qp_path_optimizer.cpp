#include "qp_path_optimizer.h"

#include <iostream>

#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {

bool QpPathOptimizer::Init(const PlannerConf &planner_conf,
                           const PathOptimizerConf &path_optimizer_conf,
                           const VehicleParams &vehicle_params) {
    sample_total_length_ = planner_conf.total_length;
    segments_num_ = path_optimizer_conf.longitudinal_sample_num;
    sample_interval_ = sample_total_length_ / segments_num_;
    SG_INFO("sample_total_length_=%lf,segments_num_=%d,sample_interval_=%lf",
            sample_total_length_, segments_num_, sample_interval_);
    lane_num = path_optimizer_conf.lane_num;
    // Number of quintic polynomial parameters
    param_num_ = path_optimizer_conf.qp_param_num;
    opt_num_ = param_num_ * segments_num_;
    inequ_num_ = path_optimizer_conf.qp_inequ_num;

    t_interval_ = path_optimizer_conf.t_interval;
    boundary_buff_ = path_optimizer_conf.boundary_buffer;
    longitudinal_safe_buff_ = path_optimizer_conf.longitudinal_safe_buffer;
    lateral_safe_buff_ = path_optimizer_conf.lateral_safe_buffer;

    // Constraint weight coefficient for curve distance
    weight1_ = path_optimizer_conf.qp_weight_1;
    // Constraint weight coefficient for first derivative of the curve
    weight2_ = path_optimizer_conf.qp_weight_2;
    // Constraint weight coefficient for second derivative of the curve
    weight3_ = path_optimizer_conf.qp_weight_3;
    // Constraint weight coefficient for thrid derivative of the curve
    weight4_ = path_optimizer_conf.qp_weight_4;

    lf_ = vehicle_params.front_edge_to_center;
    lr_ = vehicle_params.back_edge_to_center;
    vehicle_length_ = vehicle_params.vehicle_length;
    vehicle_width_ = vehicle_params.vehicle_width;

    cost_function_mat_ = Eigen::MatrixXd::Zero(opt_num_, opt_num_);

    cost_function_partial_mat_ = Eigen::MatrixXd::Zero(opt_num_, 1);
    // SG_INFO("inequ = %d", inequ_num_);
    qp_solver_.Init(opt_num_, inequ_num_);
    for (int32_t i = 0; i < opt_num_; ++i) {
        equ_constraint_.lower_bound.emplace_back(
            -std::numeric_limits<float>::max());
        equ_constraint_.upper_bound.emplace_back(
            std::numeric_limits<float>::max());
    }

    CostFunctionNormalize();

    pre_opt_var_ = std::vector<std::vector<double>>(
        lane_num, std::vector<double>(opt_num_, 0));
    return true;
}

void QpPathOptimizer::CostFunctionNormalize() {
    std::vector<double> s_order(10, 0);

    float delta_s = 1 / (sample_total_length_ / segments_num_);

    Eigen::MatrixXd first_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd second_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd third_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd forth_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    for (float s = 0; s < 1; s += delta_s) {
        s_order[0] = s;
        for (int32_t i = 1; i < 10; ++i) {
            s_order[i] = s_order[i - 1] * s;
            // SG_INFO("i = %d,s_oreder = %lf", i, s_order[i]);
        }
        first_matrix += ComputeCostFunctionFristMatrix(s_order);
        second_matrix += ComputeCostFunctionSecondMatrix(s_order);
        third_matrix += ComputeCostFunctionThirdMatrix(s_order);
        forth_matrix += ComputeCostFunctionForthMatrix(s_order);
    }
    Eigen::MatrixXd cost_matrix =
        weight1_ * first_matrix + weight2_ * second_matrix +
        weight3_ * third_matrix + weight4_ * forth_matrix;

    cost_matrix = 2 * cost_matrix;
    for (int32_t i = 0; i < segments_num_; ++i) {
        cost_function_mat_.block(i * param_num_, i * param_num_, param_num_,
                                 param_num_) = cost_matrix;
    }
}

void QpPathOptimizer::CostFunctionPartial(const DpPathData &dp_path) {
    int32_t sample_counts = dp_path.size() / segments_num_;
    double delta_s = t_interval_ / sample_interval_;
    for (int i = 0; i < segments_num_; ++i) {
        std::vector<double> path_i(param_num_, 0);
        for (int j = 0; j < sample_interval_; ++j) {
            double curr_l = dp_path[i * sample_interval_ + j].pos.l;
            double s_k = j * delta_s;
            for (int k = 0; k < param_num_; ++k) {
                path_i[k] += curr_l;
                curr_l *= s_k;
            }
        }
        for (int x = 0; x < param_num_; ++x) {
            cost_function_partial_mat_(i * param_num_ + x) = path_i[x];
        }
    }
    cost_function_partial_mat_ = -2 * weight1_ * cost_function_partial_mat_;
}

void QpPathOptimizer::CalculateQpBoundaryConstraint(const SLPoint &ref_point,
                                                    double *left_boundary,
                                                    double *right_boundary) {
    // 1 get road boundary
    // double left_width =
    // left_navigable_area_->GetLaneLineBoundary(ref_point.s); double
    // right_width =
    //     right_navigable_area_->GetLaneLineBoundary(ref_point.s);

    double left_width = left_boundary_;
    double right_width = right_boundary_;

    double adc_width_half = vehicle_width_ / 2;
    double left_road_boundary = left_width - adc_width_half - boundary_buff_;
    double right_road_boundary = right_width + adc_width_half + boundary_buff_;
    // left_road_boundary = 0.05;
    // right_road_boundary = -0.05;
    // SG_INFO("left_road_boundary = %lf,right_road_boundary = %lf",
    //         left_road_boundary, right_road_boundary);
    // 2 get static obstacle boundary

    for (const Obstacle &obstacle : (*obstacles_)) {
        if (obstacle.IsStatic() == true) {
            CalculateStaticObsConstraint(ref_point, obstacle.GetObsSlBoundary(),
                                         &left_road_boundary,
                                         &right_road_boundary);
        }
    }
    // 3 get dynamic obstacle boundary
    CalculateDynamicObsConstraint(ref_point, &left_road_boundary,
                                  &right_road_boundary);
    *left_boundary = left_road_boundary;
    *right_boundary = right_road_boundary;
}
void QpPathOptimizer::CalculateStaticObsConstraint(
    const SLPoint &ref_point, const SLStaticBoundary &obstacle,
    double *left_boundary, double *right_boundary) {
    // 1 Determine whether the obstacle is near the reference point
    if (ref_point.s - longitudinal_safe_buff_ > obstacle.end_s ||
        ref_point.s + longitudinal_safe_buff_ / 2 < obstacle.start_s) {
        return;
    }
    // 2 Find the obstacle boundary
    if (obstacle.start_l > ref_point.l) {
        *left_boundary = std::min(*left_boundary, obstacle.start_l);
        *left_boundary =
            *left_boundary - vehicle_width_ / 2 - lateral_safe_buff_;
        return;
    }

    if (obstacle.end_l < ref_point.l) {
        *right_boundary = std::max(*right_boundary, obstacle.end_l);
        *right_boundary =
            *right_boundary + vehicle_width_ / 2 + lateral_safe_buff_;
        return;
    }

    // 3 in case point in obstacle
    if (std::abs(obstacle.start_l - ref_point.l) <
        std::abs(obstacle.end_l - ref_point.l)) {
        *left_boundary = std::min(*left_boundary, obstacle.start_l);
        *left_boundary =
            *left_boundary - vehicle_width_ / 2 - lateral_safe_buff_;
    } else {
        *right_boundary = std::max(*right_boundary, obstacle.end_l);
        *right_boundary =
            *right_boundary + vehicle_width_ / 2 + lateral_safe_buff_;
    }
}

void QpPathOptimizer::CalculateDynamicObsConstraint(const SLPoint &ref_point,
                                                    double *left_boundary,
                                                    double *right_boundary) {
    for (const Obstacle &dynamic_obs : (*obstacles_)) {
        if (dynamic_obs.IsStatic()) {
            continue;
        }
        // 0 get current time polygon of dynamic obstacle
        double heuristic_time = 0;
        if (!heuristic_speed_data_->EvaluateTimeByS(ref_point.s,
                                                    &heuristic_time)) {
            continue;
        }
        PathPoint obs_point_by_time;
        if (!dynamic_obs.GetPointAtTime(heuristic_time, &obs_point_by_time)) {
            continue;
        }
        double obs_s = 0;
        double obs_l = 0;
        reference_line_->XyToSl(obs_point_by_time.position_m.x,
                                obs_point_by_time.position_m.y, &obs_s, &obs_l);

        // Box2d obstalce_box = Box2d(Vec2d(obs_s, obs_l), 0,
        // dynamic_obs.Length(),
        //                            dynamic_obs.Width());
        Box2d obstalce_box =
            Box2d(Vec2d(obs_s, obs_l), 0, dynamic_obs.Length() + 10,
                  dynamic_obs.Width());
        Polygon2d poly_obs = Polygon2d(obstalce_box);

        // 1 Determine whether the obstacle is near the reference point
        double adc_front_s = ref_point.s + vehicle_length_ / 2;
        double adc_rear_s = ref_point.s - vehicle_length_ / 2;
        if (adc_front_s < poly_obs.MinX() || adc_rear_s > poly_obs.MaxX()) {
            continue;
        }
        // LineSegment2d ref_vertical_line(Vec2d(ref_point.s, *left_boundary),
        //                                 Vec2d(ref_point.s, *right_boundary));
        LineSegment2d ref_vertical_line(Vec2d(ref_point.s, 20),
                                        Vec2d(ref_point.s, -20));
        Vec2d intersect_point{0, 0};
        // // 2 The obstacle is to the right of the reference point
        if (ref_point.l > poly_obs.MaxY()) {
            for (const LineSegment2d &poly_line : poly_obs.line_segments()) {
                if (poly_line.HasIntersectWithPoint(ref_vertical_line,
                                                    &intersect_point)) {
                    *right_boundary =
                        std::max(*right_boundary, intersect_point.y() +
                                                      lateral_safe_buff_ +
                                                      vehicle_width_ / 2);
                }
            }
        }

        // // 3 The obstacle is to the left of the reference point
        if (ref_point.l < poly_obs.MinY()) {
            for (const LineSegment2d &poly_line : poly_obs.line_segments()) {
                if (poly_line.HasIntersectWithPoint(ref_vertical_line,
                                                    &intersect_point)) {
                    *left_boundary =
                        std::min(*left_boundary, intersect_point.y() -
                                                     lateral_safe_buff_ -
                                                     vehicle_width_ / 2);
                }
            }
        }
    }
}

void QpPathOptimizer::AddBoundaryConstraint(const DpPathData &dp_path) {
    double half_adc_width = vehicle_width_ / 2;
    bool is_blocked = false;
    lf_ = 0;
    for (int i = 0; i < segments_num_; ++i) {
        for (int j = 0; j < sample_interval_; ++j) {
            double s_k = SacleDownToNormalized(j);
            double s_k_n = s_k;
            double s_k_n_1 = 1;
            std::vector<double> boundary_constraint(opt_num_, 0);

            int32_t count = i * sample_interval_ + j;
            double left_boundary =
                left_navigable_area_->GetLaneLineBoundaryByIndex(count);
            double right_boundary =
                right_navigable_area_->GetLaneLineBoundaryByIndex(count);
            if (left_boundary <= right_boundary) {
                is_blocked = true;
                break;
            }
            boundary_constraint[i * param_num_] = 1;
            for (int k = 1; k < param_num_; ++k) {
                boundary_constraint[i * param_num_ + k] =
                    s_k_n + k * lf_ * s_k_n_1;
                s_k_n_1 = s_k_n;
                s_k_n = s_k_n * s_k;
            }

            if (dp_path[i * sample_interval_ + j].pos.l > left_boundary) {
                left_boundary = dp_path[i * sample_interval_ + j].pos.l + 0.01;
            }
            if (dp_path[i * sample_interval_ + j].pos.l < right_boundary) {
                right_boundary = dp_path[i * sample_interval_ + j].pos.l - 0.01;
            }

            // int32_t count = i * sample_interval_ + j;
            // double left_boundary = 0;
            // double right_boundary = 0;
            // CalculateQpBoundaryConstraint(dp_path[count].pos, &left_boundary,
            //                               &right_boundary);

            // if (count < 3) {
            //     left_boundary =
            //         left_navigable_area_->GetLaneLineBoundaryByIndex(0);
            //     right_boundary =
            //         right_navigable_area_->GetLaneLineBoundaryByIndex(0);
            // }
            // CalculateQpBoundaryConstraint(dp_path[count].pos, &left_boundary,
            //                               &right_boundary);

            if (decide_status_ == DecideStatus::LANE_FOLLOW) {
                left_boundary = 0.01;
                right_boundary = -0.01;
            }

            inequ_constraint_.constraint_mat.emplace_back(boundary_constraint);
            inequ_constraint_.lower_bound.emplace_back(right_boundary);
            inequ_constraint_.upper_bound.emplace_back(left_boundary);
        }
        if (is_blocked) {
            break;
        }
    }
}

bool QpPathOptimizer::AddSegmentsSmoothConstraint(const int32_t n0,
                                                  const int32_t n1) {
    if (n0 < 0 || (n0 + 1 != n1) || n1 > segments_num_ - 1) {
        SG_INFO("n0 or n1 maybe wrong");
        return false;
    }
    // 1 add point smooth
    std::vector<double> add_constraints0(opt_num_, 0);
    double t0 = 1;
    double t1 = 0;

    for (int i = 0; i < param_num_; ++i) {
        add_constraints0[n0 * param_num_ + i] = t0;
        t0 = t0 * t_interval_;
    }
    add_constraints0[n1 * param_num_] = -1;

    inequ_constraint_.constraint_mat.emplace_back(add_constraints0);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);
    // 2 add derivate smooth
    t0 = 1;
    std::vector<double> add_constraints1(opt_num_, 0.0);
    for (int i = 1; i < param_num_; ++i) {
        add_constraints1[n0 * param_num_ + i] = i * t0;
        t0 = t0 * t_interval_;
    }
    add_constraints1[n1 * param_num_ + 1] = -1;

    inequ_constraint_.constraint_mat.emplace_back(add_constraints1);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_ * 10);
    inequ_constraint_.upper_bound.emplace_back(epsilon_ * 10);
    // 3 add second derivate smooth
    t0 = 1;
    std::vector<double> add_constraints2(opt_num_, 0.0);
    for (int i = 2; i < param_num_; ++i) {
        add_constraints2[n0 * param_num_ + i] = i * (i - 1) * t0;
        t0 *= t_interval_;
    }
    add_constraints2[n1 * param_num_ + 2] = -2;

    inequ_constraint_.constraint_mat.emplace_back(add_constraints2);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_ * 10);
    inequ_constraint_.upper_bound.emplace_back(epsilon_ * 10);

    return true;
}

bool QpPathOptimizer::AddPointConstraint(const double s, const double l,
                                         const double theta) {
    uint32_t curr_seg = std::floor(s / sample_interval_);
    // uint32_t curr_seg = 0;
    double s_fmod = std::fmod(s, sample_interval_) / sample_interval_;
    // SG_ERROR("s_fmod=%f", s_fmod);
    double s_k = 1;
    std::vector<double> add_constraints(opt_num_, 0.0);
    // add_constraints[0] = 1;
    for (int i = 0; i < param_num_; ++i) {
        add_constraints[curr_seg * param_num_ + i] = s_k;
        s_k *= s;
    }

    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(l - epsilon_);
    inequ_constraint_.upper_bound.emplace_back(l + epsilon_);

    // inequ_constraint_.lower_bound.emplace_back(l);
    // inequ_constraint_.upper_bound.emplace_back(l);

    s_k = 1;
    std::vector<double> add_yaw_constraints(opt_num_, 0.0);
    // add_yaw_constraints[1] = 1;
    for (int i = 1; i < param_num_; ++i) {
        add_yaw_constraints[curr_seg * param_num_ + i] = i * s_k;
        s_k *= s;
    }

    inequ_constraint_.constraint_mat.emplace_back(add_yaw_constraints);
    // inequ_constraint_.lower_bound.emplace_back(l - epsilon_);
    // inequ_constraint_.upper_bound.emplace_back(l + epsilon_);
    // if (std::fabs(l) < 3.0) {
    // inequ_constraint_.lower_bound.emplace_back(std::tan(theta) - 0.3);
    // inequ_constraint_.upper_bound.emplace_back(std::tan(theta) + 0.3);
    // } else {

    inequ_constraint_.lower_bound.emplace_back(std::tan(theta) - 0.3);
    inequ_constraint_.upper_bound.emplace_back(std::tan(theta) + 0.3);
    // }

    // SG_ERROR("bound=%f,theta=%f", std::tan(theta), theta);
    return true;
}

bool QpPathOptimizer::AddPointConstraint(const double s, const double l) {
    uint32_t curr_seg = std::floor(s / sample_interval_);
    double s_fmod = std::fmod(s, sample_interval_) / sample_interval_;
    double s_k = 1;
    std::vector<double> add_constraints(opt_num_, 0.0);
    for (int i = 0; i < param_num_; ++i) {
        add_constraints[curr_seg * param_num_ + i] = s_k;
        s_k *= s_fmod;
    }
    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(l - epsilon_);
    inequ_constraint_.upper_bound.emplace_back(l + epsilon_);
    return true;
}

void QpPathOptimizer::AddComfortConstraint(const SpeedFromVeh &speed_veh) {
    if (!is_st_inited_ || speed_opt_vars_.empty()) {
        for (int i = 0; inequ_constraint_.constraint_mat.size() < inequ_num_;
             ++i) {
            std::vector<double> null_comfort_constraint(opt_num_, 0);
            inequ_constraint_.constraint_mat.emplace_back(
                null_comfort_constraint);
            double jerk_limit = 0.2;
            inequ_constraint_.lower_bound.emplace_back(-jerk_limit);
            inequ_constraint_.upper_bound.emplace_back(jerk_limit);
        }
        return;
    }

    // for (int i = 0; i < speed_opt_vars_.size(); ++i) {
    //     SG_INFO("speed_opt_vars_[%d]=%lf", i, speed_opt_vars_[i]);
    // }

    std::map<double, QuinticPolynomialCurve> st_curve;

    double time_seg_length = time_length_ / speed_seg_num_;
    for (int32_t i = 0; i < speed_seg_num_; ++i) {
        std::vector<double> speed_curve_coefs(speed_param_num_, 0);
        for (int32_t j = 0; j < speed_param_num_; ++j) {
            speed_curve_coefs[j] = speed_opt_vars_[i * speed_param_num_ + j];
        }
        QuinticPolynomialCurve speed_curve(speed_curve_coefs);
        double s_seg_max = speed_curve.CalcuteCurveValue(time_seg_length);
        st_curve[s_seg_max] = speed_curve;
    }

    double s_compensate = speed_veh.speed_mps * 0.1;
    auto curve_iter = st_curve.begin();
    double delta_s = 1 / sample_interval_;
    for (int i = 0; i < segments_num_; ++i) {
        for (double s = 0; s < 1; s += delta_s) {
            double s_st = (i + s) * sample_interval_ + s_compensate;
            // SG_INFO("s_st=%lf,curve_iter->first=%lf", s_st,
            // curve_iter->first);
            if (s_st > curve_iter->first) {
                ++curve_iter;
            }

            double dsdt = 0.0;
            double d2sdt2 = 0.0;
            double d3sdt3 = 0.0;
            if (curve_iter == st_curve.end()) {
                --curve_iter;
                dsdt = curve_iter->second.CalcuteDerivativeCurveValue(
                    time_seg_length);
                d2sdt2 = curve_iter->second.CalcuteSecDerivativeCurveValue(
                    time_seg_length);
                d3sdt3 = curve_iter->second.CalcuteThdDerivativeCurveValue(
                    time_seg_length);
                // SG_INFO("end--d3sdt3=%lf,t=%lf", d3sdt3, time_seg_length);
            } else {
                for (double t = 0.0; t < time_seg_length; t += 0.1) {
                    double s_search = curve_iter->second.CalcuteCurveValue(t);
                    // SG_INFO("s_search=%lf,s_st=%lf", s_search, s_st);
                    if (s_search > s_st) {
                        dsdt =
                            curve_iter->second.CalcuteDerivativeCurveValue(t);
                        d2sdt2 =
                            curve_iter->second.CalcuteSecDerivativeCurveValue(
                                t);
                        d3sdt3 =
                            curve_iter->second.CalcuteThdDerivativeCurveValue(
                                t);
                        // SG_INFO("not--d3sdt3=%lf,t=%lf", d3sdt3, t);
                        break;
                    }
                }
            }

            // d3l/ds3 =
            // f'''(s) * g'3(t) + 3 * f''(s) * g'(t) * g''(t) + f'(s) * g'''(t)

            // SG_INFO("s_st=%lf,dsdt=%lf,d2sdt2=%lf,d3sdt3=%lf", s_st, dsdt,
            //         d2sdt2, d3sdt3);
            std::vector<double> comfort_constraint(opt_num_, 0);

            comfort_constraint[i * param_num_ + 0] = 0;
            comfort_constraint[i * param_num_ + 1] = d3sdt3;
            comfort_constraint[i * param_num_ + 2] =
                3 * 2 * dsdt * d2sdt2 + 2 * s * d3sdt3;
            comfort_constraint[i * param_num_ + 3] =
                6 * std::pow(dsdt, 3) + 3 * 6 * s * dsdt * d2sdt2 +
                3 * std::pow(s, 2) * d3sdt3;
            comfort_constraint[i * param_num_ + 4] =
                24 * s * std::pow(dsdt, 3) +
                3 * 12 * std::pow(s, 2) * dsdt * d2sdt2 +
                4 * std::pow(s, 3) * d3sdt3;
            comfort_constraint[i * param_num_ + 5] =
                60 * std::pow(s, 2) * std::pow(dsdt, 3) +
                3 * 20 * std::pow(s, 3) * dsdt * d2sdt2 +
                5 * std::pow(s, 4) * d3sdt3;

            inequ_constraint_.constraint_mat.emplace_back(comfort_constraint);

            // for (int k = 0; k < 6; ++k) {
            //     SG_INFO("comfort_constraint[%d]=%lf", i * param_num_ + k,
            //             comfort_constraint[i * param_num_ + k]);
            // }

            double jerk_limit = 200;
            inequ_constraint_.lower_bound.emplace_back(-jerk_limit);
            inequ_constraint_.upper_bound.emplace_back(jerk_limit);
        }
    }
}

bool QpPathOptimizer::GetDiscretePath(
    const std::vector<QuinticPolynomialCurve> &path_curve_opt,
    std::vector<PathPoint> *path_opt) {
    double delta_s = 0.5;
    for (double s = 0; s < sample_total_length_; s += delta_s) {
        int32_t curr_seg = std::floor(s / sample_total_length_);

        double curr_s = std::fmod(s, sample_interval_) / sample_interval_;
        PathPoint curr_point;
        curr_point.position_m.x = s;
        curr_point.position_m.y =
            path_curve_opt[curr_seg].CalcuteCurveValue(curr_s);
        curr_point.theta_rad = std::atan(
            path_curve_opt[curr_seg].CalcuteDerivativeCurveValue(curr_s));
        path_opt->emplace_back(curr_point);
    }
    return true;
}

void QpPathOptimizer::QpPathSolver(
    const DpPathData &dp_path, const ReferenceLine &reference_line,
    const SpeedData &heuristic_speed_data,
    const std::vector<Obstacle> &obstacle, const int32_t lane_id,
    const std::pair<LaneLine, LaneLine> &navigable_area,
    const SpeedFromVeh &speed_veh, std::vector<double> *opt_var,
    const DecideStatus &decide_status) {
    reference_line_ = &reference_line;
    heuristic_speed_data_ = &heuristic_speed_data;
    left_navigable_area_ = &navigable_area.first;
    right_navigable_area_ = &navigable_area.second;
    obstacles_ = &obstacle;
    decide_status_ = decide_status;
    // 1. Get cost function mat
    CostFunctionPartial(dp_path);
    // 2. Add continuity constraint
    ConstraintClear();

    // 3. Add start point constraint
    // AddPointConstraint(0, 0);
    double adc_start_l = dp_path[0].pos.l;
    double adc_start_s = dp_path[0].pos.s;

    // SG_INFO("adc_start_s=%lf,adc_start_l=%lf", adc_start_s, adc_start_l);
    AddPointConstraint(adc_start_s, adc_start_l, dp_path[0].theta);

    for (int32_t i = 0; i < segments_num_ - 1; ++i) {
        AddSegmentsSmoothConstraint(i, i + 1);
    }

    // AddPointConstraint(adc_start_s, adc_start_l);

    // 4. Add boundary constraint
    AddBoundaryConstraint(dp_path);
    // std::cout << cost_function_mat_ << std::endl;
    // std::cout << cost_function_partial_mat_ << std::endl;
    // for (int i = 0; i < inequ_constraint_.lower_bound.size(); ++i) {
    //     SG_INFO("lb%d = %lf", i, inequ_constraint_.lower_bound[i]);
    // }
    // for (int i = 0; i < inequ_constraint_.upper_bound.size(); ++i) {
    //     SG_INFO("ub%d = %lf", i, inequ_constraint_.upper_bound[i]);
    // }

    // 5. Add boundary constraint
    // AddComfortConstraint(speed_veh);

    // 6. qp solver
    int mat_size = inequ_constraint_.constraint_mat.size();
    inequ_num_ = inequ_constraint_.lower_bound.size();
    SG_INFO("mat_size=%d,inequ_num_=%d", mat_size, inequ_num_);
    if (!is_used_init_solver_) {
        if (!qp_solver_.Solver(cost_function_mat_, cost_function_partial_mat_,
                               equ_constraint_, inequ_constraint_, opt_var)) {
            SG_ERROR("qp path solver failed");
            for (int32_t i = 0; i < opt_num_; ++i) {
                opt_var->emplace_back(pre_opt_var_[lane_id][i]);
            }
        }
        is_used_init_solver_ = true;
    } else {
        if (!qp_solver_.HotSolver(cost_function_partial_mat_, equ_constraint_,
                                  inequ_constraint_, opt_var)) {
            SG_ERROR("qp path solver failed");
            for (int32_t i = 0; i < opt_num_; ++i) {
                opt_var->emplace_back(pre_opt_var_[lane_id][i]);
            }
        }
    }
    pre_opt_var_[lane_id] = *opt_var;
    // for (int i = 0; i < pre_opt_var_[lane_id].size(); ++i) {
    //     SG_INFO("qppathopt%d=%lf", i, pre_opt_var_[lane_id][i]);
    // }
}

void QpPathOptimizer::ConstraintClear() {
    inequ_constraint_.constraint_mat.clear();
    inequ_constraint_.lower_bound.clear();
    inequ_constraint_.upper_bound.clear();
}
}  // namespace planning_lib
}  // namespace jarvis