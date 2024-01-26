#include "dp_path_optimizer.h"

#include <algorithm>
#include <list>
#include <string>

#include "common.h"
#include "dp_path_map_node.h"
#include "line_segment2d.h"
#include "polygon2d.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {

bool IsChangeLanePath() {
    return false;
}
bool IsClearToChangeLane() {
    return false;
}
bool IsSidePass() {
    return false;
}
bool IsSidePassToLeft() {
    return false;
}
bool IsSidePassToRight() {
    return false;
}
bool DpPathOptimizer::LateralUniformSlice(
    const double sample_left_boundary, const double sample_right_boundary,
    const double sample_num, std::vector<double> *sample_lateral_l) {
    if (!sample_lateral_l || sample_num == 0) {
        return false;
    }
    double delta_l =
        (sample_left_boundary - sample_right_boundary) / (sample_num - 1);

    double start_l = 0;
    int32_t count = 0;

    for (int32_t i = 0; i < sample_num; ++i) {
        sample_lateral_l->emplace_back(sample_right_boundary + i * delta_l);
    }

    for (int i = 1; i < sample_num; ++i) {
        if ((*sample_lateral_l)[i - 1] < 0 && (*sample_lateral_l)[i] > 0) {
            if (std::fabs((*sample_lateral_l)[i - 1]) >
                std::fabs((*sample_lateral_l)[i])) {
                (*sample_lateral_l)[i] = 0;
            } else {
                (*sample_lateral_l)[i - 1] = 0;
            }
        }
    }
    return true;
}

bool DpPathOptimizer::Init(const PlannerConf &planner_conf,
                           const PathOptimizerConf &path_optimizer_conf,
                           const VehicleParams &vehicle_params) {
    sample_distance_ = planner_conf.total_length;
    lateral_sample_num_ = path_optimizer_conf.lateral_sample_num;
    longitudinal_sample_num_ = path_optimizer_conf.longitudinal_sample_num;
    lgt_sampling_interval_ = sample_distance_ / longitudinal_sample_num_;

    boundary_buffer_ = path_optimizer_conf.boundary_buffer;
    offset_to_other_referenceline_ =
        path_optimizer_conf.offset_to_other_referenceline;
    sige_pass_distance_ = path_optimizer_conf.sige_pass_distance;
    vehicle_width_ = vehicle_params.vehicle_width;
    dp_curve_info_.Init(path_optimizer_conf, vehicle_params);
    return true;
}
bool DpPathOptimizer::ConstructDpGraphMap(
    std::vector<std::vector<SLPoint>> *dp_graph_map) {
    double accmulated_s = init_sl_point_.s;
    double prev_s = accmulated_s;
    double lgt_total_length = accmulated_s + sample_distance_;

    double adc_width_half = vehicle_width_ / 2.0;

    // 1. calculate current dp path graph map

    for (int32_t i = 0; accmulated_s < lgt_total_length; ++i) {
        accmulated_s += lgt_sampling_interval_;

        if (accmulated_s + lgt_sampling_interval_ / 2.0 > lgt_total_length) {
            accmulated_s = lgt_total_length;
        }

        double current_s = accmulated_s;

        const double min_lgt_sample_len = 1.0;

        if (std::fabs(current_s - prev_s) < min_lgt_sample_len) {
            continue;
        }

        // double left_width =
        //     left_navigable_area_->GetLaneLineBoundary(current_s);
        // double right_width =
        //     right_navigable_area_->GetLaneLineBoundary(current_s);

        double left_width = left_boundary_;
        double right_width = right_boundary_;

        double right_width_space =
            right_width + adc_width_half + boundary_buffer_;
        double left_width_space =
            left_width - adc_width_half - boundary_buffer_;

        double lat_sample_unit_l = 1.2 / (lateral_sample_num_ - 1);

        // Lane change is not supported
        if (IsChangeLanePath() && IsClearToChangeLane()) {
            lat_sample_unit_l = 1.0;
        }
        double lat_sample_l_range =
            lat_sample_unit_l * (lateral_sample_num_ - 1);
        double sample_right_boundary = right_width_space;
        double sample_left_boundary = left_width_space;

        const double lateral_migration_dist = 1.75;
        if (IsChangeLanePath() ||
            std::fabs(init_sl_point_.l > lateral_migration_dist)) {
            sample_right_boundary =
                std::fmin(-right_width_space, init_sl_point_.l);
            sample_left_boundary =
                std::fmax(left_width_space, init_sl_point_.l);
        }

        std::vector<double> sample_lateral_l;
        if (IsChangeLanePath() && IsClearToChangeLane()) {
            sample_lateral_l.push_back(offset_to_other_referenceline_);
        } else if (IsSidePass()) {
            if (IsSidePassToLeft()) {
                sample_lateral_l.push_back(left_width_space +
                                           sige_pass_distance_);
            }
            if (IsSidePassToRight()) {
                sample_lateral_l.push_back(-left_width_space -
                                           sige_pass_distance_);
            }
        } else {
            LateralUniformSlice(sample_left_boundary, sample_right_boundary,
                                lateral_sample_num_, &sample_lateral_l);
        }

        std::vector<SLPoint> lateral_sample_slpoint;
        for (double sample_l : sample_lateral_l) {
            lateral_sample_slpoint.push_back(SLPoint{current_s, sample_l});
        }
        dp_graph_map->push_back(lateral_sample_slpoint);
    }
    return true;
}
bool DpPathOptimizer::DpPathCalculateMinCostNode(
    const DpCurveCost &curve_cost,
    const std::list<DpPathMapNode> &pre_node_list, DpPathMapNode *curr_node) {
    for (const DpPathMapNode &pre_node : pre_node_list) {
        QuinticPolynomialCurve curr_curve(pre_node.sl_point_, 0, 0,
                                          curr_node->sl_point_, 0, 0);

        DpCost curr_cost = curve_cost.CalculateCurveCost(
            curr_curve, pre_node.sl_point_.s, curr_node->sl_point_.s);
        curr_cost += pre_node.min_cost_;

        curr_node->UpdateCost(&pre_node, curr_curve, curr_cost);
    }
    return true;
}
DpCost DpPathOptimizer::OptimalPath(
    const ReferenceLine &reference_line, const SpeedData &heuristic_speed_data,
    const std::vector<Obstacle> &obstacle,
    const std::pair<LaneLine, LaneLine> &navigable_area, DpPathData *dp_path,
    std::string &debuginfo) {
    reference_line_ = &reference_line;
    heuristic_speed_data_ = &heuristic_speed_data;

    left_navigable_area_ = &navigable_area.first;
    right_navigable_area_ = &navigable_area.second;

    init_sl_point_.s = 0;
    init_sl_point_.l = 0;

    std::vector<std::vector<SLPoint>> dp_graph_map;
    dp_graph_map.emplace_back(std::vector<SLPoint>(1, init_sl_point_));
    ConstructDpGraphMap(&dp_graph_map);

    // for (int i = 0; i < dp_graph_map.size(); ++i) {
    //     for (int j = 0; j < dp_graph_map[i].size(); ++j) {
    //         SG_INFO("i=%d,s=%lf,l=%lf", i, dp_graph_map[i][j].s,
    //                 dp_graph_map[i][j].l);
    //     }
    // }

    debuginfo += "{dp_map:";
    for (int i = 0; i < dp_graph_map.size(); ++i) {
        for (int j = 0; j < dp_graph_map[i].size(); ++j) {
            // SG_INFO("dp_map_i=%d,s=%lf,l=%lf", i, dp_graph_map[i][j].s,
            //         dp_graph_map[i][j].l);
            debuginfo = debuginfo + "(" + std::to_string(dp_graph_map[i][j].s) +
                        "," + std::to_string(dp_graph_map[i][j].l) + "),";
        }
    }
    debuginfo += "dp_map_end}";

    std::string obs_sl_str = "{obs_sl_line:";

    std::vector<Obstacle> obstacle_for_path;
    for (const Obstacle &obs : obstacle) {
        if (obs.Behavior() == Obstacle::BEHAVIORTYPE::IGNORE ||
            obs.Behavior() == Obstacle::BEHAVIORTYPE::FOLLOW) {
            continue;
        }
        SLStaticBoundary obs_sl = obs.GetObsSlBoundary();
        Vec2d p0(obs_sl.start_s, obs_sl.start_l);
        Vec2d p1(obs_sl.start_s, obs_sl.end_l);
        Vec2d p2(obs_sl.end_s, obs_sl.start_l);
        Vec2d p3(obs_sl.end_s, obs_sl.end_l);
        obs_sl_str += "<";
        obs_sl_str +=
            "(" + std::to_string(p0.x()) + "," + std::to_string(p0.y()) + "),";
        obs_sl_str +=
            "(" + std::to_string(p1.x()) + "," + std::to_string(p1.y()) + "),";
        obs_sl_str +=
            "(" + std::to_string(p2.x()) + "," + std::to_string(p2.y()) + "),";
        obs_sl_str +=
            "(" + std::to_string(p3.x()) + "," + std::to_string(p3.y()) + ")";
        obs_sl_str += ">";

        obstacle_for_path.emplace_back(obs);
    }
    obs_sl_str += "obs_sl_line_end}";

    debuginfo += obs_sl_str;

    dp_curve_info_.Construct(*heuristic_speed_data_, reference_line_,
                             obstacle_for_path, init_sl_point_);
    std::list<std::list<DpPathMapNode>> dp_path_map_node;

    for (int32_t i = 0; i < dp_graph_map.size(); ++i) {
        std::list<DpPathMapNode> dp_nodes;
        if (i == 0) {
            for (SLPoint &dp_point : dp_graph_map[i]) {
                DpPathMapNode node_tmp(dp_point, nullptr, 0);
                dp_nodes.emplace_back(node_tmp);
            }
        } else {
            for (SLPoint &dp_point : dp_graph_map[i]) {
                DpPathMapNode node_tmp(dp_point, nullptr, MAXFLOAT);
                DpPathCalculateMinCostNode(dp_curve_info_,
                                           dp_path_map_node.back(), &node_tmp);
                dp_nodes.emplace_back(node_tmp);
            }
        }
        dp_path_map_node.emplace_back(dp_nodes);
    }
    std::list<DpPathMapNode> back_nodes = dp_path_map_node.back();
    const DpPathMapNode *back_min_cost_nodes = &back_nodes.front();

    for (const DpPathMapNode &ite : back_nodes) {
        if (back_min_cost_nodes->min_cost_ > ite.min_cost_) {
            back_min_cost_nodes = &ite;
        }
    }
    const DpPathMapNode *back_node_tmp = back_min_cost_nodes;
    DPPathPoint dp_path_point_tmp{0, 0, 0, 0};

    double curr_lever_len = sample_distance_ - lgt_sampling_interval_;

    DpCost optimal_path_cost = 0;
    optimal_path_cost += back_node_tmp->min_cost_;

    for (int32_t i = 0; i < dp_path_map_node.size(); ++i) {
        for (int j = lgt_sampling_interval_; j > 0; --j) {
            dp_path_point_tmp.pos =
                SLPoint{j + curr_lever_len,
                        back_node_tmp->min_cost_curve_.CalcuteCurveValue(j)};
            dp_path_point_tmp.dot_l =
                back_node_tmp->min_cost_curve_.CalcuteDerivativeCurveValue(j);
            dp_path_point_tmp.ddot_l =
                back_node_tmp->min_cost_curve_.CalcuteSecDerivativeCurveValue(
                    j);
            dp_path_point_tmp.dddot_l =
                back_node_tmp->min_cost_curve_.CalcuteThdDerivativeCurveValue(
                    j);
            dp_path->push_back(dp_path_point_tmp);
        }

        back_node_tmp = back_node_tmp->prev_min_cost_node_;
        curr_lever_len -= lgt_sampling_interval_;
        optimal_path_cost += back_node_tmp->min_cost_;

        if (back_node_tmp->prev_min_cost_node_ == nullptr) {
            break;
        }
    }

    DPPathPoint dp_path_point_init = dp_path->front();

    dp_path_point_init.pos = init_sl_point_;
    dp_path->push_back(dp_path_point_init);
    std::reverse(dp_path->begin(), dp_path->end());

    double last_s = dp_path->back().pos.s;
    double last_l = dp_path->back().pos.l;

    double dist = 1e10;

    double middle_s = (*dp_path)[dp_path->size() / 2].pos.s;
    double middle_l = (*dp_path)[dp_path->size() / 2].pos.l;
    for (int i = 0; i < reference_line_->CenterLines().size(); ++i) {
        double ml =
            reference_line_->CenterLines()[i].GetLaneLineBoundary(middle_s);

        if (dist > std::fabs(ml - middle_l)) {
            dist = std::fabs(ml - middle_l);
            curr_lane_id_ = i;
        }
    }

    return optimal_path_cost;
}
}  // namespace planning_lib
}  // namespace jarvis