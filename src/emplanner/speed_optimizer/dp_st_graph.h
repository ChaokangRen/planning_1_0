#pragma once
#include <vector>

#include "common.h"
#include "obstacle.h"
#include "speed_data.h"
#include "st_boundary.h"
#include "st_boundary_mapper.h"
#include "st_graph_point.h"
namespace jarvis {
namespace planning_lib {
class DpStGraph {
public:
    DpStGraph() = default;

    bool Init(const PlannerConf &planner_conf,
              const SpeedOptimizerConf &speed_optimizer_conf,
              const VehicleParams &vehicle_params);

    DpStGraph(const std::vector<StBoundary> &st_boundarys,
              const std::vector<Obstacle> &obstacles,
              const TrajectoryPoint &init_point,
              const PlannerConf &planner_conf,
              const SpeedOptimizerConf &speed_optimizer_conf,
              const VehicleParams &vehicle_params);

    bool Search(const std::vector<PathPoint> &path_points,
                SpeedData *speed_vector, double *total_cost);

    void SetLcLimitSpeed(double lc_limit_v) {
        lc_limit_speed_ = lc_limit_v;
    }

private:
    bool InitCostTable();

    void CalculateCostAt(const uint32_t c, const uint32_t r);

    void GetRowRange(const StGraphPoint &point, int *highest_row,
                     int *lowest_row);

    double GetStObstacleCost(const StGraphPoint &st_graph_point);

    double GetSpeedCost(const STPoint &first, const STPoint &second,
                        const float speed_limit);

    double GetAccelCostByThreePoints(const STPoint &first,
                                     const STPoint &second,
                                     const STPoint &thrid);

    double GetJerkCostByFourPoints(const STPoint &first, const STPoint &second,
                                   const STPoint &third, const STPoint &fourth);

    double GetAccelCostByTwoPoints(const float pre_speed,
                                   const STPoint &pre_point,
                                   const STPoint &curr_point);

    double GetJerkCostByTwoPoints(const float pre_speed, const float pre_acc,
                                  const STPoint &pre_point,
                                  const STPoint &curr_point);

    bool HasObstacleNearCurrPoint(const double curr_t,
                                  std::vector<int32_t> *obs_counts);

    bool HasCollision(const STPoint &st_point);

    double CalculateStObstacleCost(const std::vector<int32_t> &obs_counts,
                                   const double curr_s, const double curr_t,
                                   const int32_t index_t);

    bool CalculateTotalCost();

    bool CheckOverlapOnDpStGraph(const StGraphPoint &p1,
                                 const StGraphPoint &p2);

    float CalculateEdgeCost(const STPoint &first, const STPoint &second,
                            const STPoint &third, const STPoint &forth,
                            const float speed_limit);

    float CalculateEdgeCostForSecondCol(const uint32_t row,
                                        const float speed_limit);

    float CalculateEdgeCostForThirdCol(const uint32_t row,
                                       const uint32_t pre_row,
                                       const float speed_limit);

    float GetJerkCostByThreePoints(const float first_speed,
                                   const STPoint &first, const STPoint &second,
                                   const STPoint &third);

    bool GetSpeedLimits(const std::vector<PathPoint> &path_points);

    bool IsNeedFollowSpeed(double &follow_speed);

private:
    TrajectoryPoint init_point_;

    std::vector<std::vector<StGraphPoint>> cost_table_;

    std::vector<StBoundary> st_boundarys_;

    std::vector<std::vector<std::pair<double, double>>> obs_boundry_;

    std::vector<const Obstacle *> obstacles_;

    std::vector<double> speed_limit_vec_;

    int32_t dimension_s_ = 0;

    int32_t dimension_t_ = 0;

    float unit_s_ = 0.0;

    float unit_t_ = 0.0;

    float safe_time_buffer_ = 0.0;

    float speed_range_buffer_ = 0.0;

    float safe_distance_ = 0.0;

    float obstacle_weight_ = 0.0;  // could adjust

    float speed_weight_ = 0.0;

    float accel_weight_ = 0.0;

    float jerk_weight_ = 0.0;

    float speed_limit_ = 0;

    double max_acceleration_ = 0.0;

    double max_deceleration_ = 0.0;

    double front_edge_to_center_ = 0.0;

    double back_edge_to_center_ = 0.0;

    double max_centric_acc_limit_ = 0.0;

    double minimal_kappa_ = 0.0;

    double lc_limit_speed_;
};
}  // namespace planning_lib
}  // namespace jarvis