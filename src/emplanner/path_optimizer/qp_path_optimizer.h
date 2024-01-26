#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <limits>
#include <vector>

#include "common.h"
#include "polygon2d.h"
#include "qpoases_solver.h"
#include "quintic_polynomial_curve.h"
#include "reference_line.h"
#include "speed_data.h"

namespace jarvis {
namespace planning_lib {
class QpPathOptimizer {
public:
    QpPathOptimizer() = default;

    void QpPathSolver(const DpPathData &dp_path,
                      const ReferenceLine &reference_line,
                      const SpeedData &heuristic_speed_data,
                      const std::vector<Obstacle> &obstacle,
                      const int32_t lane_id,
                      const std::pair<LaneLine, LaneLine> &navigable_area,
                      const SpeedFromVeh &speed_veh,
                      std::vector<double> *opt_var,
                      const DecideStatus &decide_status);

    bool Init(const PlannerConf &planner_conf,
              const PathOptimizerConf &path_optimizer_conf,
              const VehicleParams &vehicle_params);

    void SetLeftBoundary(const double &lc_ubound) {
        left_boundary_ = lc_ubound;
    }
    void SetRightBoundary(const double &lc_lbound) {
        right_boundary_ = lc_lbound;
    }

    void SetSpeedOptVars(const std::vector<double> &speed_opt_vars,
                         const double time_length, const int32_t speed_seg_num,
                         const int32_t speed_param_num) {
        is_st_inited_ = true;
        speed_opt_vars_ = speed_opt_vars;
        time_length_ = time_length;
        speed_seg_num_ = speed_seg_num;
        speed_param_num_ = speed_param_num;
    };

private:
    void CostFunctionNormalize();

    void CostFunctionPartial(const DpPathData &dp_path);

    void CalculateQpBoundaryConstraint(const SLPoint &ref_point,
                                       double *left_boundary,
                                       double *right_boundary);

    void CalculateStaticObsConstraint(const SLPoint &ref_point,
                                      const SLStaticBoundary &obstacle,
                                      double *left_boundary,
                                      double *right_boundary);

    void CalculateDynamicObsConstraint(const SLPoint &ref_point,
                                       double *left_boundary,
                                       double *right_boundary);

    void AddBoundaryConstraint(const DpPathData &dp_path);

    void AddComfortConstraint(const SpeedFromVeh &speed_veh);

    double SacleDownToNormalized(double s_k) {
        return (int(s_k) % int(sample_interval_)) / sample_interval_;
    }

    bool AddSegmentsSmoothConstraint(const int32_t n0, const int32_t n1);

    bool AddPointConstraint(const double s, const double l, const double theta);

    bool AddPointConstraint(const double s, const double l);

    bool GetDiscretePath(
        const std::vector<QuinticPolynomialCurve> &path_curve_opt,
        std::vector<PathPoint> *path_opt);

    void ConstraintClear();

private:
    const ReferenceLine *reference_line_ = nullptr;

    const SpeedData *heuristic_speed_data_ = nullptr;

    Constraint inequ_constraint_;

    Constraint equ_constraint_;

    // cost function matrix
    Eigen::MatrixXd cost_function_mat_;

    Eigen::MatrixXd cost_function_partial_mat_;

    QpoasesSolver qp_solver_;

    const double epsilon_ = 0.001;
    bool is_used_init_solver_ = false;

    std::vector<std::vector<double>> pre_opt_var_;

    const LaneLine *left_navigable_area_;
    const LaneLine *right_navigable_area_;
    const std::vector<Obstacle> *obstacles_;

    DecideStatus decide_status_;

    double sample_total_length_ = 0.0;
    double sample_interval_ = 0.0;
    int32_t segments_num_ = 0;
    int32_t lane_num = 0;
    int32_t opt_num_ = 0;
    // Number of quintic polynomial parameters
    int32_t param_num_ = 0;
    int32_t inequ_num_ = 0;
    double t_interval_ = 0.0;
    double boundary_buff_ = 0.0;
    double longitudinal_safe_buff_ = 0.0;
    double lateral_safe_buff_ = 0.0;

    // Constraint weight coefficient for curve distance
    double weight1_ = 0.0;
    // Constraint weight coefficient for first derivative of the curve
    double weight2_ = 0.0;
    // Constraint weight coefficient for second derivative of the curve
    double weight3_ = 0.0;
    // Constraint weight coefficient for thrid derivative of the curve
    double weight4_ = 0.0;

    double lf_ = 0.0;
    double lr_ = 0.0;
    double vehicle_length_ = 0.0;
    double vehicle_width_ = 0.0;

    double left_boundary_ = 2.0;
    double right_boundary_ = -2.0;

    bool is_st_inited_ = false;
    std::vector<double> speed_opt_vars_;
    double time_length_ = 0.0;
    int32_t speed_seg_num_ = 0;
    int32_t speed_param_num_ = 0;
};
}  // namespace planning_lib
}  // namespace jarvis