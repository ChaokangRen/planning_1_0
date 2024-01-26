#include "qpoases_solver.h"

#include <iostream>

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {
bool QpoasesSolver::Init(int32_t opt_nums, int32_t inequ_nums) {
    opt_nums_ = opt_nums;
    inequ_nums_ = inequ_nums;
    qp_solver_ = QProblem(opt_nums_, inequ_nums_);
    options_.printLevel = PL_NONE;
    qp_solver_.setOptions(options_);

    qp_h_ = new real_t[opt_nums_ * opt_nums_];

    qp_g_ = new real_t[opt_nums_];

    qp_a_ = new real_t[opt_nums_ * inequ_nums_];

    qp_lba_ = new real_t[inequ_nums_];

    qp_uba_ = new real_t[inequ_nums_];

    qp_lb_ = new real_t[opt_nums_];

    qp_ub_ = new real_t[opt_nums_];

    opt_vars_ = new real_t[opt_nums_];
    for (int32_t i = 0; i < opt_nums_; ++i) {
        opt_vars_[i] = 0;
    }

    nwsr_ = 1000;

    is_inited = true;
    return true;
}

QpoasesSolver::~QpoasesSolver() {
    if (is_inited == true) {
        delete qp_h_;
        delete qp_g_;
        delete qp_a_;
        delete qp_lb_;
        delete qp_ub_;
        delete qp_lba_;
        delete qp_uba_;
        delete opt_vars_;
    }
}

bool QpoasesSolver::Solver(const Eigen::MatrixXd qp_h,
                           const Eigen::MatrixXd qp_g,
                           const Constraint &equ_constraint,
                           const Constraint &inequ_constraint,
                           std::vector<double> *opt_var) {
    for (int i = 0; i < opt_nums_; ++i) {
        for (int j = 0; j < opt_nums_; ++j) {
            qp_h_[i * opt_nums_ + j] = qp_h(i, j);
        }
        qp_g_[i] = qp_g(i, 0);
        qp_lb_[i] = equ_constraint.lower_bound[i];
        qp_ub_[i] = equ_constraint.upper_bound[i];
    }
    for (int i = 0; i < inequ_nums_; ++i) {
        for (int j = 0; j < opt_nums_; ++j) {
            qp_a_[i * opt_nums_ + j] = inequ_constraint.constraint_mat[i][j];
        }
        qp_lba_[i] = inequ_constraint.lower_bound[i];
        qp_uba_[i] = inequ_constraint.upper_bound[i];
    }
    nwsr_ = 1000;
    if (SUCCESSFUL_RETURN != qp_solver_.init(qp_h_, qp_g_, qp_a_, qp_lb_,
                                             qp_ub_, qp_lba_, qp_uba_, nwsr_)) {
        SG_ERROR("qp init solver failed");
        for (int i = 0; i < opt_nums_; ++i) {
            opt_var->emplace_back(opt_vars_[i]);
        }
        return false;
    }
    qp_solver_.getPrimalSolution(opt_vars_);
    for (int i = 0; i < opt_nums_; ++i) {
        opt_var->emplace_back(opt_vars_[i]);
    }
    return true;
}
bool QpoasesSolver::HotSolver(const Eigen::MatrixXd qp_g,
                              const Constraint &equ_constraint,
                              const Constraint &inequ_constraint,
                              std::vector<double> *opt_var) {
    for (int i = 0; i < opt_nums_; ++i) {
        qp_g_[i] = qp_g(i, 0);
        qp_lb_[i] = equ_constraint.lower_bound[i];
        qp_ub_[i] = equ_constraint.upper_bound[i];
    }
    for (int i = 0; i < inequ_nums_; ++i) {
        qp_lba_[i] = inequ_constraint.lower_bound[i];
        qp_uba_[i] = inequ_constraint.upper_bound[i];
    }

    nwsr_ = 1000;
    if (SUCCESSFUL_RETURN !=
        qp_solver_.hotstart(qp_g_, qp_lb_, qp_ub_, qp_lba_, qp_uba_, nwsr_)) {
        SG_ERROR("qp hot solver failed");
        return false;
    }

    qp_solver_.getPrimalSolution(opt_vars_);

    for (int i = 0; i < opt_nums_; ++i) {
        opt_var->emplace_back(opt_vars_[i]);
    }
    return true;
}
}  // namespace planning_lib
}  // namespace jarvis