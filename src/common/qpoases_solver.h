#pragma once

#include <Eigen/Eigen>
#include <qpOASES.hpp>
#include <vector>

#include "common.h"

USING_NAMESPACE_QPOASES

namespace jarvis {
namespace planning_lib {
class QpoasesSolver {
public:
    QpoasesSolver() = default;

    QpoasesSolver(int32_t opt_nums, int32_t inequ_nums) {
        Init(opt_nums, inequ_nums);
    }

    bool Init(int32_t opt_nums, int32_t inequ_nums);

    bool Solver(const Eigen::MatrixXd qp_h, const Eigen::MatrixXd qp_g,
                const Constraint &equ_constraint,
                const Constraint &inequ_constraint,
                std::vector<double> *opt_var);

    bool HotSolver(const Eigen::MatrixXd qp_g, const Constraint &equ_constraint,
                   const Constraint &inequ_constraint,
                   std::vector<double> *opt_var);

    ~QpoasesSolver();

private:
    // The number of optimization variables
    int32_t opt_nums_;

    // The number of inequality constraint
    int32_t inequ_nums_;

    QProblem qp_solver_;

    Options options_;

    // The optimal solution of quadratic programming
    real_t *opt_vars_;

    real_t *qp_h_;

    real_t *qp_g_;

    real_t *qp_a_;

    real_t *qp_lb_;

    real_t *qp_ub_;

    real_t *qp_lba_;

    real_t *qp_uba_;

    int_t nwsr_ = 1000;

    bool is_inited = false;
};
}  // namespace planning_lib
}  // namespace jarvis