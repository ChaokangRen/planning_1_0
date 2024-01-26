#pragma once
#include <Eigen/Eigen>
#include <vector>

#include "common.h"
#include "qpoases_solver.h"
#include "speed_data.h"
#include "st_boundary.h"
#include "st_point.h"

namespace jarvis {
namespace planning_lib {
class QpSpeedOptimizer {
public:
    QpSpeedOptimizer() = default;

    bool Init(const PlannerConf &planner_conf,
              const SpeedOptimizerConf &speed_optimizer_conf,
              const VehicleParams &vehicle_params);

    bool QpSpeedSolver(const std::vector<StBoundary> &st_boundarys,
                       const SpeedData &speed_points,
                       const TrajectoryPoint &init_point,
                       std::vector<double> *opt_vars);

private:
    void CostFunctionNormalize();

    void GetCostFunctionPartial(const std::vector<STPoint> &dp_speed,
                                const SpeedData &speed_points);

    bool AddPointConstraint(const double t, const double s);

    bool AddPointDerivativeConstraint(const double t, const double velocity);

    bool AddSpeedConstraint(const double t, const double vel_low,
                            const double vel_high);

    bool AddAccelerationConstraint(const double t);

    bool AddMonotonicityConstraint(const double t1, const double t0);

    bool AddSegmentsSmoothConstraint(const int32_t n0, const int32_t n1);

    bool AddStBoundaryConstraint(const std::vector<StBoundary> &st_boundarys,
                                 const SpeedData &speed_points);

    bool AddJerkConstraint(const double t);

    bool CalCount(const double t, uint32_t &curr_seg, double &t_fmod);

    uint32_t GetTCount(const double) const;
    void ConstraintClear();

private:
    Constraint equ_constraint_;

    Constraint inequ_constraint_;

    const double epsilon_ = 0.0001;

    double max_velocity_ = 0.0;

    float time_length_ = 0.0;

    int32_t opt_num_ = 0;

    int32_t seg_num_ = 0;

    int32_t param_num_ = 0;

    int32_t inequ_num_ = 0;

    int32_t st_points_num_ = 0;

    double normal_jerk_ = 0.0;

    double upper_boundary_ = 0.0;

    double lower_boundary_ = 0.0;

    double weight1_ = 0.0;

    double weight2_ = 0.0;

    double weight3_ = 0.0;

    double weight4_ = 0.0;

    double max_acceleration_ = 0.0;

    double min_deceleration_ = 0.0;

    // cost function matrix
    Eigen::MatrixXd cost_function_mat_;

    Eigen::MatrixXd cost_function_partial_mat_;

    QpoasesSolver qp_solver_;

    bool is_used_init_solver_ = false;

    std::vector<double> pre_opt_vars_;
};
}  // namespace planning_lib
}  // namespace jarvis