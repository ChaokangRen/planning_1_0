#pragma once

#include "decider.h"
#include "lane.h"
#include "lane_change_decider.h"
#include "lateral_obstacle_decider.h"
#include "path_bounds_decider.h"
#include "path_optimizer/dp_path_optimizer.h"
#include "path_optimizer/qp_path_optimizer.h"
#include "planner_interface.h"
#include "speed_data.h"
#include "speed_optimizer/dp_speed_optimizer.h"
#include "speed_optimizer/qp_speed_optimizer.h"

namespace jarvis {
namespace planning_lib {
class EmPlanner : public PlannerInterface {
public:
    virtual ~EmPlanner() = default;

    virtual bool Init(const PlanningConf &planning_conf);

    virtual bool Execute(
        const Lane &lane, const std::vector<ReferenceLineInfo> &reference_lines,
        const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
        const LaneChangeMsg &lane_change_msg, const PnCLane &pnc_center_lines,
        const DriveState &drive_state, TrajectoryMsg *trajectory_msg,
        std::vector<Obstacle> &obstacle, std::string &debuginfo);
    int32_t GetCurrLaneId() {
        return dp_path_optimizer_.GetCurrLaneId();
    }

private:
    Decider decider_;

    PathBoundsDecider path_bounds_decider_;
    LateralObstacleDecider lateral_obs_decider_;

    const std::vector<ReferenceLineInfo> *reference_lines_ = nullptr;

    SpeedData heuristic_speed_data_;

    DpPathOptimizer dp_path_optimizer_;

    QpPathOptimizer qp_path_optimizer_;

    LaneChangeDecider lane_change_decider_;

    DpSpeedOptimizer dp_speed_optimizer_;

    QpSpeedOptimizer qp_speed_optimizer_;

    TrajectoryPoint init_point_;

    std::vector<SpeedPoint> speed_vector_;

    double trajectory_time_interval_ = 0.0;

    int32_t path_param_num_ = 0;

    int32_t path_segments_num_ = 0;

    double sample_interval_ = 0.0;

    double sample_total_length_ = 0.0;

    double time_length_ = 0.0;

    int32_t speed_seg_num_ = 0;

    int32_t speed_param_num_ = 0;

    TrajectoryMsg pre_trajectory_;

    int32_t refresh_cnt = 0;
};
}  // namespace planning_lib
}  // namespace jarvis