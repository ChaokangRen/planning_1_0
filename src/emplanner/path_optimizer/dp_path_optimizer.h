#pragma once

#include <list>
#include <vector>

#include "common.h"
#include "dp_curve_cost.h"
#include "dp_path_map_node.h"
#include "lane_line.h"
#include "polygon2d.h"
#include "reference_line.h"
#include "speed_data.h"

namespace jarvis {
namespace planning_lib {

class DpPathOptimizer {
public:
    DpPathOptimizer() = default;

    bool Init(const PlannerConf &planner_conf,
              const PathOptimizerConf &path_optimizer_conf,
              const VehicleParams &vehicle_params);

    explicit DpPathOptimizer(const ReferenceLine &reference_line,
                             const SpeedData &heuristic_speed_data)
        : reference_line_(&reference_line),
          heuristic_speed_data_(&heuristic_speed_data) {}

    DpCost OptimalPath(const ReferenceLine &reference_line,
                       const SpeedData &heuristic_speed_data,
                       const std::vector<Obstacle> &obstacle,
                       const std::pair<LaneLine, LaneLine> &navigable_area,
                       DpPathData *dp_path, std::string &debuginfo);

    int32_t GetCurrLaneId() const {
        return curr_lane_id_;
    }

    void SetLeftBoundary(const double &lc_ubound) {
        left_boundary_ = lc_ubound;
    }
    void SetRightBoundary(const double &lc_lbound) {
        right_boundary_ = lc_lbound;
    }

private:
    bool ConstructDpGraphMap(std::vector<std::vector<SLPoint>> *dp_graph_map);

    bool DpPathCalculateMinCostNode(
        const DpCurveCost &curve_cost,
        const std::list<DpPathMapNode> &pre_node_list,
        DpPathMapNode *curr_node);

    bool DpPathCalculateMinCostNode(
        const std::vector<SLStaticBoundary> &dp_static_obstacles,
        const std::vector<Polygon2d> &dp_dynamic_obstacles,
        const std::list<DpPathMapNode> &pre_node_list,
        DpPathMapNode *curr_node);

    bool LateralUniformSlice(const double sample_left_boundary,
                             const double sample_right_boundary,
                             const double sample_num,
                             std::vector<double> *sample_lateral_l);

private:
    DpCurveCost dp_curve_info_;
    SLPoint init_sl_point_;
    const LaneLine *left_navigable_area_;
    const LaneLine *right_navigable_area_;
    const ReferenceLine *reference_line_ = nullptr;
    const SpeedData *heuristic_speed_data_ = nullptr;

    double sample_distance_ = 0.0;
    int32_t lateral_sample_num_ = 0;
    double lgt_sampling_interval_ = 0.0;
    int32_t longitudinal_sample_num_ = 0;
    double boundary_buffer_ = 0.0;
    double offset_to_other_referenceline_ = 0.0;
    double sige_pass_distance_ = 0.0;
    double vehicle_width_ = 0.0;

    int32_t curr_lane_id_ = 0;

    double left_boundary_ = 1.75;
    double right_boundary_ = -1.75;
};
}  // namespace planning_lib
}  // namespace jarvis
