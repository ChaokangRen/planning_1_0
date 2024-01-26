#pragma once
// #include <Eigen/Dense>
#include <Eigen/Eigen>

#include "common.h"
#include "qpoases_solver.h"
#include "quintic_polynomial_curve.h"
#include "vec2d.h"

namespace jarvis {
namespace planning_lib {
class SmoothQpSolver {
public:
    SmoothQpSolver() = default;

    bool Init(const ReferenceLineSmoothConf &reference_line_smooth_conf);

    bool QpSolver(
        const std::vector<PathPoint> &center_laneline,
        const PosFromIns &pos_ins,
        std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
            *curve_pairs);

private:
    void CostFunctionNormalize();

    void CostFunctionPartial();

    void AddPointConstraint(const Vec2d &point, const double s);

    void AddPointsConstraint(const std::vector<PathPoint> &center_laneline);

    void AddDirectionConstraint(const double theta, const double s);

    void AddSmoothConstraint(void);

    void AddSmoothConstraint(int32_t seg_counts);

    void ConstraintClear();

private:
    // reference line that need to be optimized
    const std::vector<PathPoint> *reference_line_;

    // Equality constraints
    Constraint equ_constraint_;

    // Inequality constraint
    Constraint inequ_constraint_;

    // cost function matrix
    Eigen::MatrixXd cost_function_mat_;

    Eigen::MatrixXd cost_function_partial_mat_;

    QpoasesSolver qp_solver_;

    const double epsilon_ = 0.001;
    bool is_used_init_solver_ = false;

    // Optimized total length of reference line
    double refline_total_length_ = 0.0;
    // Number of segments
    int32_t seg_num_ = 0;
    // Number of optimized variables
    int32_t opt_num_ = 0;
    // Number of quintic polynomial parameters
    int32_t param_num_ = 0;
    // Number of constraints
    int32_t inequ_num_ = 0;
    // Constraint weight coefficient for curve distance
    double weight1_ = 0.0;
    // Constraint weight coefficient for first derivative of the curve
    double weight2_ = 0.0;
    // Constraint weight coefficient for second derivative of the curve
    double weight3_ = 0.0;
    // Constraint weight coefficient for thrid derivative of the curve
    double weight4_ = 0.0;
};

}  // namespace planning_lib
}  // namespace jarvis