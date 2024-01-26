#include "qp_speed_optimizer.h"

#include <iostream>

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

double SearchSFormSpeedData(const std::vector<SpeedPoint> speed_points,
                            const double current_t) {
    int32_t cnt = 0;
    if (current_t <= speed_points.front().t) {
        return speed_points.front().s;
    }
    if (current_t >= speed_points.back().t) {
        return speed_points.back().s;
    }
    for (int32_t i = 1; i < speed_points.size(); ++i) {
        if (speed_points[i].t > current_t) {
            cnt = i - 1;
            break;
        }
    }

    return lerp(speed_points[cnt].s, speed_points[cnt].t,
                speed_points[cnt + 1].s, speed_points[cnt + 1].t, current_t);
}

bool QpSpeedOptimizer::Init(const PlannerConf &planner_conf,
                            const SpeedOptimizerConf &speed_optimizer_conf,
                            const VehicleParams &vehicle_params) {
    max_velocity_ = speed_optimizer_conf.speed_limit;

    time_length_ = planner_conf.total_time;

    opt_num_ = speed_optimizer_conf.qp_opt_num;

    seg_num_ = speed_optimizer_conf.qp_seg_num;

    param_num_ = speed_optimizer_conf.qp_param_num;

    inequ_num_ = speed_optimizer_conf.qp_inequ_num;
    inequ_num_ = 172;
    st_points_num_ = speed_optimizer_conf.st_points_num;

    normal_jerk_ = speed_optimizer_conf.normal_jerk;

    upper_boundary_ = speed_optimizer_conf.upper_boundary;

    lower_boundary_ = speed_optimizer_conf.lower_boundary;

    weight1_ = speed_optimizer_conf.qp_weight_1;

    weight2_ = speed_optimizer_conf.qp_weight_2;

    weight3_ = speed_optimizer_conf.qp_weight_3;

    weight4_ = speed_optimizer_conf.qp_weight_4;

    max_acceleration_ = vehicle_params.max_acceleration;

    min_deceleration_ = -vehicle_params.max_deceleartion;

    SG_INFO("time_length_=%lf", time_length_);

    SG_INFO(
        "opt_num_=%d,seg_num_=%d,param_num_=%d,inequ_num_=%d,st_points_num_=%d",
        opt_num_, seg_num_, param_num_, inequ_num_, st_points_num_);

    SG_INFO(
        "max_velocity_=%lf,normal_jerk_=%lf,upper_boundary_=%lf,lower_boundary_"
        "=%lf",
        max_velocity_, normal_jerk_, upper_boundary_, lower_boundary_);

    SG_INFO("weight1_=%lf,weight2_=%lf,weight3_=%lf,weight4_=%lf", weight1_,
            weight2_, weight3_, weight4_);

    SG_INFO("max_acceleration_=%lf,min_deceleration_=%lf", max_acceleration_,
            min_deceleration_);
    cost_function_mat_ = Eigen::MatrixXd::Zero(opt_num_, opt_num_);
    cost_function_partial_mat_ = Eigen::MatrixXd::Zero(opt_num_, 1);

    qp_solver_.Init(opt_num_, inequ_num_);
    pre_opt_vars_ = std::vector<double>(opt_num_, 0);
    SG_INFO("opt num = %d,inequ num = %d", opt_num_, inequ_num_);
    for (int32_t i = 0; i < opt_num_; ++i) {
        equ_constraint_.lower_bound.emplace_back(
            -std::numeric_limits<float>::max());
        equ_constraint_.upper_bound.emplace_back(
            std::numeric_limits<float>::max());
    }
    CostFunctionNormalize();
    return true;
}
bool QpSpeedOptimizer::QpSpeedSolver(
    const std::vector<StBoundary> &st_boundarys, const SpeedData &speed_points,
    const TrajectoryPoint &init_point, std::vector<double> *opt_vars) {
    std::vector<STPoint> dp_speed;
    ConstraintClear();

    for (const SpeedPoint &speed_point : speed_points.SpeedVector()) {
        dp_speed.emplace_back(STPoint{speed_point.s, speed_point.t});
    }
    GetCostFunctionPartial(dp_speed, speed_points);

    AddPointConstraint(0, epsilon_);
    for (int i = 1; i < seg_num_; ++i) {
        AddSegmentsSmoothConstraint(i - 1, i);
    }
    double init_high_vel = init_point.velocity_mps + 0.31;
    double init_low_vel = init_point.velocity_mps - 0.37;
    AddSpeedConstraint(0, init_low_vel, init_high_vel);
    for (float t = 0.25; t < dp_speed.back().t(); t += 0.25) {
        AddMonotonicityConstraint(t, t - 0.25);
    }
    AddStBoundaryConstraint(st_boundarys, speed_points);

    for (float t = 0.0; t < dp_speed.back().t(); t += 0.05) {
        AddJerkConstraint(t);
    }
    // SG_INFO("qp speed size = %d", inequ_constraint_.lower_bound.size());
    bool is_qp_slover_failed = true;

    if (!qp_solver_.Solver(cost_function_mat_, cost_function_partial_mat_,
                           equ_constraint_, inequ_constraint_, opt_vars)) {
        SG_ERROR("qp speed solver failed!");
    } else {
        is_qp_slover_failed = false;
        // SG_INFO("qp speed solver successed!");
    }
    // for (int32_t i = 0; i < (*opt_vars).size(); ++i) {
    //     SG_INFO("i = %d,val = %lf", i, (*opt_vars)[i]);
    // }

    if (is_qp_slover_failed == true) {
        (*opt_vars) = std::vector<double>(opt_num_, 0);
        return false;
    }

    return true;
}

// bool QpSpeedOptimizer::QpSpeedSolver(
//     const std::vector<StBoundary> &st_boundarys, const SpeedData
//     &speed_points, const TrajectoryPoint &init_point, std::vector<double>
//     *opt_vars) { std::vector<STPoint> dp_speed; for (const SpeedPoint
//     &speed_point : speed_points.SpeedVector()) {
//         dp_speed.emplace_back(STPoint{speed_point.s, speed_point.t});
//     }

//     GetCostFunctionPartial(dp_speed);
//     ConstraintClear();

//     // for (int i = 1; i < seg_num_; ++i) {
//     //     AddSegmentsSmoothConstraint(i - 1, i);
//     // }
//     // AddPointConstraint(0, epsilon_);
//     // double init_high_vel = init_point.velocity_mps + 0.05;
//     // double init_low_vel = init_point.velocity_mps - 0.0;
//     // double vel_error_limit = 3.0;
//     // double vel_buff = 0.5;
//     // if (speed_points.SpeedVector().empty() == false) {
//     //     float vel_error = speed_points.SpeedVector().back().velocity -
//     //                       init_point.velocity_mps;
//     //     if (vel_error > vel_error_limit) {
//     //         if (init_point.velocity_mps < 7) {
//     //             init_high_vel += vel_buff;
//     //         } else {
//     //             init_high_vel += 0.15;
//     //         }

//     //     } else if (vel_error < -vel_error_limit) {
//     //         // init_low_vel -= vel_buff;
//     //         init_low_vel -= 0.0;
//     //     }
//     // }
//     // AddSpeedConstraint(0, init_low_vel, init_high_vel);
//     // // int32_t speed_count_start = inequ_constraint_.upper_bound.size();
//     // // for (int32_t i = 1; i < speed_points.SpeedVector().size(); ++i) {
//     // //     double speed_error = 0.5;
//     // //     double lo_speed = std::fmax(
//     // //         0.0, speed_points.SpeedVector()[i].velocity - speed_error);
//     // //     double hi_speed =
//     // //         std::fmin(max_velocity_,
//     // //                   speed_points.SpeedVector()[i].velocity +
//     // speed_error);

//     // //     AddSpeedConstraint(speed_points.SpeedVector()[i].t, lo_speed,
//     // //     hi_speed);
//     // // }

//     // // for (int32_t i = 1; i < speed_points.SpeedVector().size() - 1; ++i)
//     {
//     // //     AddPointConstraint(speed_points.SpeedVector()[i].t,
//     // //                        speed_points.SpeedVector()[i].s);
//     // // }
//     // int32_t speed_count_end = inequ_constraint_.upper_bound.size();
//     // for (float t = 0.5; t < dp_speed.back().t(); t += 0.5) {
//     //     AddMonotonicityConstraint(t, t - 0.25);
//     //     // AddAccelerationConstraint(t);
//     //     // AddSpeedConstraint(t, 0, max_velocity_);
//     // }
//     int32_t jerk_start_cnt = inequ_constraint_.lower_bound.size();
//     // // SG_INFO("qp speed inequ size = %d", jerk_start_cnt);
//     // // for (float t = 0.5; t < dp_speed.back().t(); t += 0.5) {
//     // //     AddJerkConstraint(t);
//     // // }
//     int32_t jerk_end_cnt = inequ_constraint_.lower_bound.size();

//     // AddStBoundaryConstraint(st_boundarys, speed_points);
//     // SG_INFO("qp speed inequ size = %d",
//     // inequ_constraint_.lower_bound.size());

//     for (int32_t i = 0; i < inequ_constraint_.lower_bound.size(); ++i) {
//         SG_INFO("i = %d,lo = %lf,hi = %lf", i,
//         inequ_constraint_.lower_bound[i],
//                 inequ_constraint_.upper_bound[i]);
//     }
//     bool is_qp_slover_failed = true;
//     if (!is_used_init_solver_) {
//         if (!qp_solver_.Solver(cost_function_mat_,
//         cost_function_partial_mat_,
//                                equ_constraint_, inequ_constraint_, opt_vars))
//                                {
//             SG_ERROR("qp speed solver failed!");
//         } else {
//             is_qp_slover_failed = false;
//         }

//         is_used_init_solver_ = true;
//     } else {
//         if (!qp_solver_.HotSolver(cost_function_partial_mat_,
//         equ_constraint_,
//                                   inequ_constraint_, opt_vars)) {
//             for (int32_t i = 0; i < 2; ++i) {
//                 for (int32_t j = jerk_start_cnt; j < jerk_end_cnt; ++j) {
//                     inequ_constraint_.lower_bound[j] = -(i + 1) *
//                     normal_jerk_; inequ_constraint_.upper_bound[j] = (i + 1)
//                     * normal_jerk_;
//                 }
//                 if (qp_solver_.HotSolver(cost_function_partial_mat_,
//                                          equ_constraint_, inequ_constraint_,
//                                          opt_vars)) {
//                     SG_WARN("qp speed solver successed!");
//                     is_qp_slover_failed = false;
//                     break;
//                 } else {
//                     SG_ERROR("qp speed solver failed!");
//                 }
//             }
//         } else {
//             SG_INFO("qp speed solver successed!");
//         }
//     }
//     if (is_qp_slover_failed == true) {
//         (*opt_vars) = std::vector<double>(opt_num_, 0);
//     }

//     return true;
// }

void QpSpeedOptimizer::CostFunctionNormalize() {
    std::vector<double> s_order(10, 0);

    float time_interval = time_length_ / seg_num_;
    float delta_s = 0.1;

    Eigen::MatrixXd first_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd second_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd thrid_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd forth_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    for (float s = 0; s < time_interval; s += delta_s) {
        s_order[0] = s;
        for (int32_t i = 1; i < 10; ++i) {
            s_order[i] = s_order[i - 1] * s;
        }
        first_matrix += ComputeCostFunctionFristMatrix(s_order);
        second_matrix += ComputeCostFunctionSecondMatrix(s_order);
        thrid_matrix += ComputeCostFunctionThirdMatrix(s_order);
        forth_matrix += ComputeCostFunctionForthMatrix(s_order);
    }
    Eigen::MatrixXd cost_matrix =
        weight1_ * first_matrix + weight2_ * second_matrix +
        weight3_ * thrid_matrix + weight4_ * forth_matrix;
    cost_matrix = 2 * cost_matrix;

    for (int32_t i = 0; i < seg_num_; ++i) {
        cost_function_mat_.block(i * param_num_, i * param_num_, param_num_,
                                 param_num_) = cost_matrix;
    }
}
void QpSpeedOptimizer::GetCostFunctionPartial(
    const std::vector<STPoint> &dp_speed, const SpeedData &speed_points) {
    int32_t sample_counts = time_length_ / seg_num_;

    float time_interval = time_length_ / seg_num_;
    float delta_t = 0.1;

    for (int32_t i = 0; i < seg_num_; ++i) {
        std::vector<double> part_vec_sum(param_num_, 0);
        for (float t_k = 0; t_k < time_interval; t_k += delta_t) {
            float current_t = i * time_interval + t_k;
            double part_value =
                SearchSFormSpeedData(speed_points.SpeedVector(), current_t);

            // SG_INFO("t_k = %lf,delta_t = %lf,ct = %lf,val = %lf", t_k,
            // delta_t,
            //         current_t, part_value);
            for (int k = 0; k < param_num_; ++k) {
                part_vec_sum[k] += part_value;
                part_value *= t_k;
            }
        }
        for (int32_t x = 0; x < param_num_; ++x) {
            cost_function_partial_mat_(i * param_num_ + x) = part_vec_sum[x];
        }
    }
    cost_function_partial_mat_ = -2 * weight1_ * cost_function_partial_mat_;
}

uint32_t QpSpeedOptimizer::GetTCount(const double t) const {
    uint32_t curr_seg = static_cast<int>(t / (time_length_ / seg_num_));
    if (curr_seg >= seg_num_) {
        curr_seg = seg_num_ - 1;
    }
    return curr_seg;
}
bool QpSpeedOptimizer::CalCount(const double t, uint32_t &curr_seg,
                                double &t_fmod) {
    curr_seg = static_cast<int>(t / (time_length_ / seg_num_));
    t_fmod = std::fmod(t, time_length_ / seg_num_);
    if (curr_seg >= seg_num_) {
        curr_seg = seg_num_ - 1;
        t_fmod += time_length_ / seg_num_;
    }
    return true;
}

bool QpSpeedOptimizer::AddPointConstraint(const double t, const double s) {
    uint32_t curr_seg = GetTCount(t);
    double t_fmod = std::fmod(t, time_length_ / seg_num_);
    CalCount(t, curr_seg, t_fmod);
    double t_s = 1;
    std::vector<double> add_constraints(opt_num_, 0.0);
    for (int i = 0; i < param_num_; ++i) {
        add_constraints[curr_seg * param_num_ + i] = t_s;
        t_s *= t_fmod;
    }
    double point_epsilon_ = 0.1;
    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(s);
    inequ_constraint_.upper_bound.emplace_back(s + point_epsilon_);
    return true;
}

bool QpSpeedOptimizer::AddPointDerivativeConstraint(const double t,
                                                    const double velocity) {
    uint32_t curr_seg = GetTCount(t);
    double t_fmod = std::fmod(t, time_length_ / seg_num_);
    double t_s = 1;
    std::vector<double> add_constraints(opt_num_, 0.0);
    for (int i = 1; i < param_num_; ++i) {
        add_constraints[curr_seg * param_num_ + i] = i * t_s;
        t_s *= t_fmod;
    }
    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(velocity - epsilon_);
    inequ_constraint_.upper_bound.emplace_back(velocity + epsilon_);
    return true;
}

bool QpSpeedOptimizer::AddSpeedConstraint(const double t, const double vel_low,
                                          const double vel_high) {
    uint32_t curr_seg = GetTCount(t);
    double t_fmod = std::fmod(t, time_length_ / seg_num_);

    CalCount(t, curr_seg, t_fmod);
    std::vector<double> add_constraints(opt_num_, 0.0);
    double t_s = 1;
    for (int i = 1; i < param_num_; ++i) {
        add_constraints[curr_seg * param_num_ + i] = i * t_s;
        t_s *= t_fmod;
    }

    // SG_INFO("vel low = %f,vel hi = %lf", vel_low, vel_high);
    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(vel_low);
    inequ_constraint_.upper_bound.emplace_back(vel_high);
    return true;
}

bool QpSpeedOptimizer::AddJerkConstraint(const double t) {
    uint32_t curr_seg = GetTCount(t);
    double t_fmod = std::fmod(t, time_length_ / seg_num_);
    std::vector<double> add_constraints(opt_num_, 0.0);
    double t_s = 1;
    for (int i = 2; i < param_num_; ++i) {
        add_constraints[curr_seg * param_num_ + i] = i * (i - 1) * t_s;
        t_s *= t_fmod;
    }
    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(-normal_jerk_);
    inequ_constraint_.upper_bound.emplace_back(normal_jerk_);
    return true;
}

bool QpSpeedOptimizer::AddAccelerationConstraint(const double t) {
    uint32_t curr_seg = GetTCount(t);
    double t_fmod = std::fmod(t, time_length_ / seg_num_);
    std::vector<double> add_constraints(opt_num_, 0.0);
    double t_s = 1;
    for (int i = 2; i < param_num_; ++i) {
        add_constraints[curr_seg * param_num_ + i] = -i * (i - 1) * t_s;
        t_s *= t_fmod;
    }
    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(min_deceleration_);
    inequ_constraint_.upper_bound.emplace_back(max_acceleration_);
    return true;
}

bool QpSpeedOptimizer::AddMonotonicityConstraint(const double t1,
                                                 const double t0) {
    if (t1 < t0) {
        SG_INFO("t1 < t0 ,wrong!");
        return false;
    }
    uint32_t curr_seg0 = GetTCount(t0);
    double t_fmod0 = std::fmod(t0, time_length_ / seg_num_);
    uint32_t curr_seg1 = GetTCount(t1);
    double t_fmod1 = std::fmod(t1, time_length_ / seg_num_);
    // SG_INFO("t1 = %lf,t0 = %lf,seg0 = %d,seg1 = %d,d0 = %lf,d1 = %lf", t1,
    // t0,
    //         curr_seg0, curr_seg1, t_fmod0, t_fmod1);
    std::vector<double> add_constraints(opt_num_, 0.0);
    if (curr_seg0 == curr_seg1) {
        for (int i = 1; i < param_num_; ++i) {
            add_constraints[curr_seg0 * param_num_ + i] =
                std::pow(t_fmod1, i) - std::pow(t_fmod0, i);
        }
    } else {
        for (int i = 0; i < param_num_; ++i) {
            add_constraints[curr_seg0 * param_num_ + i] = -std::pow(t_fmod0, i);
            add_constraints[curr_seg1 * param_num_ + i] = std::pow(t_fmod1, i);
        }
    }
    inequ_constraint_.constraint_mat.emplace_back(add_constraints);
    inequ_constraint_.lower_bound.emplace_back(0);
    inequ_constraint_.upper_bound.emplace_back(
        std::numeric_limits<float>::infinity());
    return true;
}
bool QpSpeedOptimizer::AddSegmentsSmoothConstraint(const int32_t n0,
                                                   const int32_t n1) {
    if (n0 < 0 || (n0 + 1 != n1) || n1 > seg_num_ - 1) {
        SG_INFO("n0 or n1 maybe wrong");
        return false;
    }
    // 1 add point smooth
    std::vector<double> add_constraints0(opt_num_, 0);
    double t0 = 1;
    const double dec_t = time_length_ / seg_num_;
    double t1 = 0;
    for (int i = 0; i < param_num_; ++i) {
        add_constraints0[n0 * param_num_ + i] = t0;
        t0 *= dec_t;
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
        t0 *= dec_t;
    }
    add_constraints1[n1 * param_num_ + 1] = -1;

    inequ_constraint_.constraint_mat.emplace_back(add_constraints1);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);
    // 3 add second derivate smooth
    t0 = 1;
    std::vector<double> add_constraints2(opt_num_, 0.0);
    for (int i = 2; i < param_num_; ++i) {
        add_constraints2[n0 * param_num_ + i] = i * (i - 1) * t0;
        t0 *= dec_t;
    }
    add_constraints2[n1 * param_num_ + 2] = -2;

    inequ_constraint_.constraint_mat.emplace_back(add_constraints2);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);
    return true;
}

bool QpSpeedOptimizer::AddStBoundaryConstraint(
    const std::vector<StBoundary> &st_boundarys,
    const SpeedData &speed_points) {
    double high_limits = upper_boundary_;
    double low_limits = lower_boundary_;
    for (float t = epsilon_; t < time_length_; t += 0.25) {
        double current_s = SearchSFormSpeedData(speed_points.SpeedVector(), t);
        for (const StBoundary &boundary : st_boundarys) {
            if (t >= boundary.MinT() && t <= boundary.MaxT()) {
                double s_upper = upper_boundary_;
                double s_lower = lower_boundary_;
                boundary.GetBoundarySRange(t, &s_upper, &s_lower);

                // high_limits = std::min(high_limits, s_upper);
                // low_limits = std::max(low_limits, s_lower);
                if (current_s <= s_lower) {
                    high_limits = s_lower;
                }
                if (current_s >= s_upper) {
                    low_limits = s_upper;
                }
            }
        }
        uint32_t curr_seg = GetTCount(t);
        double t_fmod = std::fmod(t, time_length_ / seg_num_);
        double t_s = 1;
        std::vector<double> add_constraints(opt_num_, 0.0);
        for (int i = 0; i < param_num_; ++i) {
            add_constraints[curr_seg * param_num_ + i] = t_s;
            t_s *= t_fmod;
        }
        // SG_INFO("lo = %lf,hi = %lf", low_limits, high_limits);
        inequ_constraint_.constraint_mat.emplace_back(add_constraints);
        inequ_constraint_.lower_bound.emplace_back(low_limits);
        inequ_constraint_.upper_bound.emplace_back(high_limits);
    }
    return true;
}

void QpSpeedOptimizer::ConstraintClear() {
    inequ_constraint_.constraint_mat.clear();
    inequ_constraint_.lower_bound.clear();
    inequ_constraint_.upper_bound.clear();
}
}  // namespace planning_lib
}  // namespace jarvis
