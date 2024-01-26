#pragma once
#include <vector>

#include "common.h"
#include "obstacle.h"
#include "polygon2d.h"
#include "quintic_polynomial_curve.h"
#include "reference_line.h"
#include "speed_data.h"

namespace jarvis {
namespace planning_lib {
class DpCurveCost {
public:
    DpCurveCost() = default;

    explicit DpCurveCost(const SpeedData &heuristic_speed_data,
                         const ReferenceLine *reference_line,
                         const std::vector<Obstacle> &obstacles,
                         const SLPoint init_sl_point);

    bool Construct(const SpeedData &heuristic_speed_data,
                   const ReferenceLine *reference_line,
                   const std::vector<Obstacle> &obstacles,
                   const SLPoint init_sl_point);

    bool Init(const PathOptimizerConf &path_optimizer_conf,
              const VehicleParams &vehicle_params);

    DpCost CalculateCurveCost(const QuinticPolynomialCurve &curve,
                              const float start_s, const float end_s) const;

    DpCost CalculateDpPathCost(const QuinticPolynomialCurve &curve,
                               const float start_s, const float end_s) const;

    DpCost CalculateDpStaticObsCost(const QuinticPolynomialCurve &curve,
                                    const float start_s,
                                    const float end_s) const;

    DpCost CalculateDpDynamicObsCost(const QuinticPolynomialCurve &curve,
                                     const float start_s,
                                     const float end_s) const;

private:
    DpCost CalcuteSingelStaticObsCost(const SLStaticBoundary &static_obs,
                                      const double curve_s,
                                      const double curve_l) const;

    DpCost CalcuteCostBetweenObsPolygon(const Polygon2d &ego_poly,
                                        const Polygon2d &obs_poly) const;

    double Sigmoid(const double x) const {
        return 1.0 / (1.0 + std::exp(-x));
    }

    Box2d GetBoxFromSLPoint(const SLPoint &sl_point, const float dl) const;

private:
    const std::vector<Obstacle> *obstacles_;

    SpeedData heuristic_speed_data_;

    SLPoint init_sl_point_;

    const ReferenceLine *reference_line_ = nullptr;

    std::vector<std::vector<Box2d>> dynamic_obstacle_boxes_;

    uint32_t num_of_time_stamps_;

    double eval_time_interval_ = 0.0;
    float dp_path_cost_sample_dist_ = 0.0;
    float dp_path_cost_ = 0.0;
    float dp_path_dl_cost_ = 0.0;
    float dp_path_ddl_cost_ = 0.0;
    float dp_static_obs_cost_ = 0.0;
    float dp_dynamic_obs_cost_small_ = 0.0;
    float dp_dynamic_obs_cost_big_ = 0.0;
    double has_collision_cost_ = 0.0;
    double safety_distance_ = 0.0;
    double obstacle_collision_distance_ = 0.0;
    double risk_obstacle_collision_distance_ = 0.0;
    double obstacle_ignore_distance_ = 0.0;
    double static_obs_safe_ratio_ = 0.0;

    double vehicle_length_ = 0.0;
    double vehicle_width_ = 0.0;
    double front_edge_to_center_ = 0.0;
    double back_edge_to_center_ = 0.0;
    double left_edge_to_center_ = 0.0;
    double right_edge_to_center_ = 0.0;
    double rearview_mirror_width_ = 0.0;
};
}  // namespace planning_lib
}  // namespace jarvis
