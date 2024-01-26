#include "emplanner.h"

#include <cmath>

#include "quintic_polynomial_curve.h"
#include "sglog/sglog.h"
#include "sgtime/sgtime.h"

namespace jarvis {
namespace planning_lib {

PlannerInterface *PlannerInterface::CreateInstance() {
    return new EmPlanner();
}

bool EmPlanner::Init(const PlanningConf &planning_conf) {
    trajectory_time_interval_ =
        planning_conf.planner_conf.trajectory_time_interval;
    path_param_num_ = planning_conf.path_optimizer_conf.qp_param_num;
    path_segments_num_ =
        planning_conf.path_optimizer_conf.longitudinal_sample_num;
    sample_total_length_ = planning_conf.planner_conf.total_length;
    sample_interval_ = sample_total_length_ / path_segments_num_;

    time_length_ = planning_conf.planner_conf.total_time;
    speed_seg_num_ = planning_conf.speed_optimizer_conf.qp_seg_num;
    speed_param_num_ = planning_conf.speed_optimizer_conf.qp_param_num;

    dp_path_optimizer_.Init(planning_conf.planner_conf,
                            planning_conf.path_optimizer_conf,
                            planning_conf.vehicle_params);
    qp_path_optimizer_.Init(planning_conf.planner_conf,
                            planning_conf.path_optimizer_conf,
                            planning_conf.vehicle_params);
    dp_speed_optimizer_.Init(planning_conf.planner_conf,
                             planning_conf.speed_optimizer_conf,
                             planning_conf.vehicle_params);
    qp_speed_optimizer_.Init(planning_conf.planner_conf,
                             planning_conf.speed_optimizer_conf,
                             planning_conf.vehicle_params);
    init_point_.path_point.position_m.x = 0;
    init_point_.path_point.position_m.y = 0;
    init_point_.velocity_mps = 0;
    init_point_.relative_time_s = 0;

    std::vector<SpeedPoint> speed_points;
    for (float t = 0; t < time_length_; t += trajectory_time_interval_) {
        SpeedPoint point;
        point.t = t;
        point.velocity = 0.01;
        point.s = point.velocity * t;
        point.acc = 0;
        point.dot_acc = 0;
        speed_points.emplace_back(point);
    }
    heuristic_speed_data_.SetSpeedVector(speed_points);
    return true;
}

bool EmPlanner::Execute(
    const Lane &lane, const std::vector<ReferenceLineInfo> &reference_lines,
    const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
    const LaneChangeMsg &lane_change_msg, const PnCLane &pnc_center_lines,
    const DriveState &drive_state, TrajectoryMsg *trajectory_msg,
    std::vector<Obstacle> &obstacle, std::string &debuginfo) {
    DpPathData dp_path_data;
    reference_lines_ = &reference_lines;

    SpeedPoint speed_point;
    // init_point_.velocity_mps = std::round(speed_veh.speed_mps);
    init_point_.velocity_mps = speed_veh.speed_mps;
    // SG_INFO("veh speed = %lf", speed_veh.speed_mps);
    int32_t cost_min_count = 0;
    DpCost min_cost = std::numeric_limits<double>::max();
    std::vector<std::pair<LaneLine, LaneLine>> sl_navigable_areas;

    std::vector<std::vector<QuinticPolynomialCurve>> path_curves;
    std::vector<double> speed_opt_vars;
    for (int i = 0; i < reference_lines.size(); ++i) {
        sl_navigable_areas.emplace_back(
            std::make_pair(reference_lines[i].reference_line.GetSlLineInFrenet(
                               reference_lines[i].left_navigable_area),
                           reference_lines[i].reference_line.GetSlLineInFrenet(
                               reference_lines[i].right_navigable_area)));
    }
    // in this version,the adc position is start point in frenet
    SLPoint adc_point{0, 0};
    for (int i = 0; i < reference_lines.size(); ++i) {
        // add lateral obstacle decider
        lateral_obs_decider_.Process(lane, reference_lines[i].reference_line,
                                     pos_ins, speed_veh, pnc_center_lines,
                                     drive_state, adc_point, obstacle);

        decider_.Process(lane, obstacle, pos_ins, speed_veh, lane_change_msg,
                         reference_lines[i].reference_line, drive_state,
                         debuginfo);
        dp_path_optimizer_.SetLeftBoundary(decider_.GetUpBound());
        dp_path_optimizer_.SetRightBoundary(decider_.GetLowBound());
        DpPathData tmp_dp_path_data;
        DpCost curr_cost =
            dp_path_optimizer_.OptimalPath(
                reference_lines[i].reference_line, heuristic_speed_data_,
                obstacle, sl_navigable_areas[i], &tmp_dp_path_data, debuginfo) +
            reference_lines[i].reference_line.LaneCenterOffsetCost();

        debuginfo += "{dp_line:";
        for (int i = 0; i < tmp_dp_path_data.size(); ++i) {
            // SG_INFO("dps = %lf,dpl = %lf", tmp_dp_path_data[i].pos.s,
            //         tmp_dp_path_data[i].pos.l);
            debuginfo = debuginfo + "(" +
                        std::to_string(tmp_dp_path_data[i].pos.s) + "," +
                        std::to_string(tmp_dp_path_data[i].pos.l) + "),";
        }
        debuginfo += "dp_line_end}";

        path_bounds_decider_.Process(
            heuristic_speed_data_, obstacle, tmp_dp_path_data,
            &reference_lines[i].reference_line, sl_navigable_areas);

        std::vector<double> path_opt_vars;
        // for (int i = 0; i < tmp_dp_path_data.size(); ++i) {
        //     SG_INFO("dps=%lf,dpl=%lf", tmp_dp_path_data[i].pos.s,
        //             tmp_dp_path_data[i].pos.l);
        // }
        auto left_navigable_area = sl_navigable_areas[i].first;
        auto right_navigable_area = sl_navigable_areas[i].second;
        // for (int s = 0; s < 20; ++s) {
        //     double left_width = left_navigable_area.GetLaneLineBoundary(s);
        //     double right_width = right_navigable_area.GetLaneLineBoundary(s);
        //     SG_INFO("left_width=%lf,right_width=%lf", left_width,
        //     right_width);
        // }

        qp_path_optimizer_.QpPathSolver(
            tmp_dp_path_data, reference_lines[i].reference_line,
            heuristic_speed_data_, obstacle, i, sl_navigable_areas[i],
            speed_veh, &path_opt_vars, decider_.GetDecideStatus());
        double delta_s = 1 / sample_interval_;
        DpPathData path_optimizer_datas;
        std::vector<QuinticPolynomialCurve> tmp_path_curves;
        for (int32_t i = 0; i < path_segments_num_; ++i) {
            std::vector<double> coefs;
            for (int32_t j = 0; j < path_param_num_; ++j) {
                coefs.emplace_back(path_opt_vars[i * path_param_num_ + j]);
                // SG_INFO("coef%d = %lf", j, coefs.back());
            }
            QuinticPolynomialCurve dp_optimal_curve(coefs);
            for (double s = 0; s < 1; s += delta_s) {
                double l = dp_optimal_curve.CalcuteCurveValue(s);
                double dot_l = dp_optimal_curve.CalcuteDerivativeCurveValue(s);
                double ddot_l =
                    dp_optimal_curve.CalcuteSecDerivativeCurveValue(s);
                double dddot_l =
                    dp_optimal_curve.CalcuteThdDerivativeCurveValue(s);

                double real_s = i * sample_interval_ + s * sample_interval_;
                // SG_INFO("reals = %lf,l = %lf", real_s, l);

                SLPoint sl_point{real_s, l};
                DPPathPoint dp_path_point{sl_point, dot_l, ddot_l, dddot_l};

                path_optimizer_datas.emplace_back(dp_path_point);
            }
            tmp_path_curves.emplace_back(dp_optimal_curve);
        }
        path_curves.emplace_back(tmp_path_curves);
        std::vector<PathPoint> path_points;

        reference_lines[i].reference_line.GetCartesianOptimalPath(
            path_optimizer_datas, &path_points);

        // add decider
        lane_change_decider_.Process(
            pnc_center_lines, obstacle, pos_ins, speed_veh,
            reference_lines[i].reference_line, path_points);
        SG_WARN("vel_exp=%f", lane_change_decider_.GetVelExp());
        dp_speed_optimizer_.SetLcLimitSpeed(lane_change_decider_.GetVelExp());

        SpeedData dp_speed_data;
        std::vector<StBoundary> st_boundarys;

        double speed_cost = 0;

        std::vector<Obstacle> obstacle_tmp;
        if (i == 0) {
            obstacle_tmp = obstacle;
        }

        dp_speed_optimizer_.OptimalSpeed(
            path_points, obstacle_tmp, reference_lines[i].reference_line,
            init_point_, &dp_speed_data, &st_boundarys, &speed_cost, debuginfo);
        // for (int32_t i = 0; i < dp_speed_data.SpeedVector().size(); ++i) {
        //     SG_INFO("i = %d,t = %lf,s = %lf,v = %lf", i,
        //             dp_speed_data.SpeedVector()[i].t,
        //             dp_speed_data.SpeedVector()[i].s,
        //             dp_speed_data.SpeedVector()[i].velocity);
        // }

        std::vector<double> tmp_speed_opt_vars;
        qp_speed_optimizer_.QpSpeedSolver(st_boundarys, dp_speed_data,
                                          init_point_, &speed_opt_vars);
    }
    if (trajectory_msg->trajectory.size() != 0) {
        trajectory_msg->trajectory.clear();
    }
    double trajectory_time = 0;
    std::vector<TrajectoryPoint> trajectory;

    int print_count = 0;
    std::vector<SpeedPoint> speed_points;

    std::string debug_sl_line;
    std::string debug_st_line;
    debug_sl_line += "{sl_line:";
    debug_st_line += "{st_line:";
    for (int32_t i = 0; i < speed_seg_num_; ++i) {
        std::vector<double> speed_curve_coefs(speed_param_num_, 0);
        for (int32_t j = 0; j < speed_param_num_; ++j) {
            speed_curve_coefs[j] = speed_opt_vars[i * speed_param_num_ + j];
        }

        QuinticPolynomialCurve speed_curve(speed_curve_coefs);

        for (double t = 0; t < time_length_ / speed_seg_num_;
             t += trajectory_time_interval_) {
            TrajectoryPoint trajectory_point;
            double s = speed_curve.CalcuteCurveValue(t);

            // double s = speed_curve.CalcuteCurveValue(t) + 0.001;
            if (s < 0) s = 0;
            if (print_count > 0) {
                if (s - trajectory[print_count - 1].path_point.s_m < 0) {
                    s = trajectory[print_count - 1].path_point.s_m + 0.0001;
                }
            }
            print_count++;
            double ds = speed_curve.CalcuteDerivativeCurveValue(t);
            double dds = speed_curve.CalcuteSecDerivativeCurveValue(t);
            trajectory_point.path_point.s_m = s;
            trajectory_point.velocity_mps =
                speed_curve.CalcuteDerivativeCurveValue(t);

            trajectory_point.acceleration =
                speed_curve.CalcuteSecDerivativeCurveValue(t);

            trajectory_point.dot_acc =
                speed_curve.CalcuteThdDerivativeCurveValue(t);

            SpeedPoint speed_point;
            speed_point.t = trajectory_time;
            speed_point.s = s;
            speed_point.velocity = trajectory_point.velocity_mps;
            speed_point.acc = trajectory_point.acceleration;
            speed_point.dot_acc = trajectory_point.dot_acc;
            speed_points.emplace_back(speed_point);

            int32_t tmp_count = static_cast<int32_t>(s / sample_interval_);
            if (tmp_count >= path_curves[cost_min_count].size()) {
                tmp_count = path_curves[cost_min_count].size() - 1;
            }

            double nomial_s = std::fmod(s, sample_interval_) / sample_interval_;
            double l = path_curves[cost_min_count][tmp_count].CalcuteCurveValue(
                nomial_s);
            double dl = path_curves[cost_min_count][tmp_count]
                            .CalcuteDerivativeCurveValue(nomial_s);
            double ddl = path_curves[cost_min_count][tmp_count]
                             .CalcuteSecDerivativeCurveValue(nomial_s);

            std::array<double, 3> s_condition{s, ds, dds};
            std::array<double, 3> d_condition{l, dl, ddl};

            debug_sl_line +=
                "(" + std::to_string(s) + "," + std::to_string(l) + "),";
            debug_st_line += "(" + std::to_string(s) + "," +
                             std::to_string(trajectory_time) + "),";

            Vec2d xy;
            reference_lines[cost_min_count].reference_line.SLToXY(SLPoint{s, l},
                                                                  &xy);
            trajectory_time += trajectory_time_interval_;
            trajectory_point.path_point.position_m.x = xy.x();
            trajectory_point.path_point.position_m.y = xy.y();
            trajectory_point.path_point.s_m = s;
            trajectory_point.path_point.kappa = 0;
            trajectory_point.velocity_mps = trajectory_point.velocity_mps;
            trajectory_point.acceleration = trajectory_point.acceleration;
            trajectory_point.relative_time_s = trajectory_time;

            // if (print_count % 5 == 0) {
            //     SG_INFO("t = %lf,s = %lf,l = %lf,v = %lf,a = %lf",
            //             trajectory_time, s, l, trajectory_point.velocity_mps,
            //             trajectory_point.acceleration);
            // }

            trajectory.emplace_back(trajectory_point);
        }
    }

    // add debuginfo msg
    debug_sl_line += "sl_line_end}";
    debug_st_line += "st_line_end}";
    // DebugInfo debug_info;
    // debug_info.SetDebugInfo(debug_sl_line);
    // debug_info.SetDebugInfo(debug_st_line);

    debuginfo = debuginfo + debug_sl_line + debug_st_line;
    auto d = heuristic_speed_data_.GetSpeedVector();
    heuristic_speed_data_.SetSpeedVector(
        reference_lines[0].reference_line.GerReflineSpeedPoints(
            dp_path_optimizer_.GetCurrLaneId()));
    auto e = heuristic_speed_data_.GetSpeedVector();
    trajectory_msg->trajectory = trajectory;
    pre_trajectory_ = (*trajectory_msg);
    return true;
}
}  // namespace planning_lib
}  // namespace jarvis