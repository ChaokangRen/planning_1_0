#include "smooth_qp_solver.h"

#include <limits>

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

bool SmoothQpSolver::Init(
    const ReferenceLineSmoothConf &reference_line_smooth_conf) {
    refline_total_length_ = reference_line_smooth_conf.refline_total_length;
    seg_num_ = reference_line_smooth_conf.seg_num;
    param_num_ = reference_line_smooth_conf.qp_param_num;
    opt_num_ = 2 * seg_num_ * param_num_;
    inequ_num_ = reference_line_smooth_conf.qp_inequ_num;

    SG_INFO("qp smooth opt_num = %d,inequ_num = %d,seg_num = %d", opt_num_,
            inequ_num_, seg_num_);

    weight1_ = reference_line_smooth_conf.qp_weight_1;
    weight2_ = reference_line_smooth_conf.qp_weight_2;
    weight3_ = reference_line_smooth_conf.qp_weight_3;
    weight4_ = reference_line_smooth_conf.qp_weight_4;
    cost_function_mat_ = Eigen::MatrixXd::Zero(opt_num_, opt_num_);
    cost_function_partial_mat_ = Eigen::MatrixXd::Zero(opt_num_, 1);

    qp_solver_.Init(opt_num_, inequ_num_);

    for (int i = 0; i < opt_num_; ++i) {
        equ_constraint_.lower_bound.emplace_back(
            -std::numeric_limits<float>::max());
        equ_constraint_.upper_bound.emplace_back(
            std::numeric_limits<float>::max());
    }
    return true;
}

bool SmoothQpSolver::QpSolver(
    const std::vector<PathPoint> &center_laneline, const PosFromIns &pos_ins,
    std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
        *curve_pairs) {
    reference_line_ = &center_laneline;
    CostFunctionNormalize();
    CostFunctionPartial();
    ConstraintClear();
    AddSmoothConstraint();
    AddPointConstraint(Vec2d(pos_ins.position.x, pos_ins.position.y), 0);

    AddPointsConstraint(center_laneline);
    AddDirectionConstraint(pos_ins.yaw, 0);
    std::vector<double> qp_opt_var;
    // SG_INFO("smooth inequ size = %d", inequ_constraint_.lower_bound.size());
    if (!qp_solver_.Solver(cost_function_mat_, cost_function_partial_mat_,
                           equ_constraint_, inequ_constraint_, &qp_opt_var)) {
        SG_ERROR("smooth qp solver failed");
    }
    for (int i = 0; i < seg_num_; ++i) {
        std::vector<double> coef1(param_num_, 0);
        std::vector<double> coef2(param_num_, 0);
        for (int j = 0; j < param_num_; ++j) {
            coef1[j] = qp_opt_var[i * 2 * param_num_ + j];
            coef2[j] = qp_opt_var[i * 2 * param_num_ + param_num_ + j];
        }
        curve_pairs->emplace_back(std::make_pair(
            QuinticPolynomialCurve(coef1), QuinticPolynomialCurve(coef2)));
    }
    double smooth_line_theta = std::atan2(qp_opt_var[7], qp_opt_var[1]);
    return true;
}

void SmoothQpSolver::CostFunctionNormalize() {
    std::vector<double> s_order(10, 0);

    float delta_s = 1 / (refline_total_length_ / seg_num_);

    Eigen::MatrixXd first_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd second_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd thrid_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    Eigen::MatrixXd forth_matrix =
        Eigen::MatrixXd::Zero(param_num_, param_num_);
    for (float s = 0; s < 1; s += delta_s) {
        s_order[0] = s;
        for (int32_t i = 1; i < 10; ++i) {
            s_order[i] = s_order[i - 1] * s;
        }
        first_matrix += ComputeCostFunctionFristMatrix(s_order);
        second_matrix += ComputeCostFunctionSecondMatrix(s_order);
        thrid_matrix += ComputeCostFunctionThirdMatrix(s_order);
        forth_matrix += ComputeCostFunctionForthMatrix(s_order);
    }

    Eigen::MatrixXd cost_matrix =
        weight1_ * first_matrix + weight2_ * second_matrix +
        weight3_ * thrid_matrix + weight4_ * forth_matrix;

    cost_matrix = 2 * cost_matrix;

    int32_t num = 2 * param_num_;

    for (int32_t i = 0; i < seg_num_; ++i) {
        cost_function_mat_.block(i * num, i * num, param_num_, param_num_) =
            cost_matrix;
        cost_function_mat_.block(i * num + param_num_, i * num + param_num_,
                                 param_num_, param_num_) = cost_matrix;
    }
}

void SmoothQpSolver::CostFunctionPartial() {
    int32_t line_per = static_cast<int>(refline_total_length_ / seg_num_);
    int32_t seg_param = 2 * param_num_;

    for (int32_t i = 0; i < seg_num_; ++i) {
        std::vector<double> part_vec_sum(seg_param, 0);
        for (int32_t j = 0; j < line_per; ++j) {
            double curr_s = double(j) / double(line_per);
            double curr_x = (*reference_line_)[i * line_per + j].position_m.x;
            double curr_y = (*reference_line_)[i * line_per + j].position_m.y;

            for (int32_t k = 0; k < param_num_; ++k) {
                part_vec_sum[k] += curr_x;
                part_vec_sum[param_num_ + k] += curr_y;
                curr_x *= curr_s;
                curr_y *= curr_s;
            }
        }
        for (int32_t x = 0; x < param_num_; ++x) {
            cost_function_partial_mat_(i * seg_param + x, 0) = part_vec_sum[x];
            cost_function_partial_mat_(i * seg_param + param_num_ + x, 0) =
                part_vec_sum[param_num_ + x];
        }
    }
    for (int i = 0; i < cost_function_partial_mat_.size(); ++i) {
        cost_function_partial_mat_(i, 0) *= -2 * weight1_;
    }
}

void SmoothQpSolver::AddPointsConstraint(
    const std::vector<PathPoint> &center_laneline) {
    // SG_INFO("center_laneline.size() = %d,refline_total_length_ = %lf",
    //         center_laneline.size(), refline_total_length_);
    for (int32_t i = 10; i < static_cast<int32_t>(refline_total_length_);
         i += 10) {
        AddPointConstraint(Vec2d(center_laneline[i].position_m.x,
                                 center_laneline[i].position_m.y),
                           i * 1.0);
    }
}

void SmoothQpSolver::AddPointConstraint(const Vec2d &point, const double s) {
    if (s > refline_total_length_ || s < 0) {
        SG_ERROR("smooth_qp_solver_: s = %lf marbe wrong");
        return;
    }
    int32_t segments_num = std::floor(s / (refline_total_length_ / seg_num_));
    double noimal_s = std::fmod(s, (refline_total_length_ / seg_num_)) /
                      (refline_total_length_ / seg_num_);

    std::vector<double> points_constraint_x(opt_num_, 0);
    std::vector<double> points_constraint_y(opt_num_, 0);
    double curr_s = 1.0;
    for (int32_t i = 0; i < param_num_; ++i) {
        points_constraint_x[segments_num * (param_num_ * 2) + i] = curr_s;
        points_constraint_y[(segments_num * 2 * param_num_) + param_num_ + i] =
            curr_s;
        curr_s *= noimal_s;
    }
    // 1 add x constraint
    inequ_constraint_.constraint_mat.emplace_back(points_constraint_x);
    inequ_constraint_.lower_bound.emplace_back(point.x() - epsilon_);
    inequ_constraint_.upper_bound.emplace_back(point.x() + epsilon_);

    // 2 add y constraint
    inequ_constraint_.constraint_mat.emplace_back(points_constraint_y);
    inequ_constraint_.lower_bound.emplace_back(point.y() - epsilon_);
    inequ_constraint_.upper_bound.emplace_back(point.y() + epsilon_);
}
void SmoothQpSolver::AddDirectionConstraint(const double theta,
                                            const double s) {
    if (s > refline_total_length_ || s < 0) {
        SG_ERROR("smooth_qp_solver_: s = %lf marbe wrong");
        return;
    }
    double normalize_theta = theta;
    int32_t segments_num = std::floor(s / (refline_total_length_ / seg_num_));
    double noimal_s = std::fmod(s, (refline_total_length_ / seg_num_)) /
                      (refline_total_length_ / seg_num_);

    double cos_theta = std::cos(normalize_theta);
    double sin_theta = std::sin(normalize_theta);

    std::vector<double> first_order_coeff{0, 1, 2, 3, 4, 5};
    std::vector<double> vertical_component(opt_num_, 0);

    double tmp_s = 1;
    for (int i = 1; i < param_num_; ++i) {
        vertical_component[segments_num * 2 * param_num_ + i] =
            first_order_coeff[i] * sin_theta * tmp_s;
        vertical_component[segments_num * 2 * param_num_ + param_num_ + i] =
            -first_order_coeff[i] * cos_theta * tmp_s;
        tmp_s *= noimal_s;
    }

    inequ_constraint_.constraint_mat.emplace_back(vertical_component);

    double allow_theta = 0.1;
    inequ_constraint_.lower_bound.emplace_back(-allow_theta);
    inequ_constraint_.upper_bound.emplace_back(allow_theta);

    Vec2d sgn(1, 1);
    if (normalize_theta > 0 && normalize_theta <= M_PI / 2) {
        sgn.set_x(1);
        sgn.set_y(1);
    } else if (normalize_theta > M_PI / 2 && normalize_theta <= M_PI) {
        sgn.set_x(-1);
        sgn.set_y(1);
    } else if (normalize_theta > M_PI && normalize_theta <= 3 * M_PI / 2) {
        sgn.set_x(-1);
        sgn.set_y(-1);
    } else if (normalize_theta > 3 * M_PI / 2 && normalize_theta <= 2 * M_PI) {
        sgn.set_x(1);
        sgn.set_y(-1);
    }

    std::vector<double> direction_vec(opt_num_);
    for (int i = 1; i < param_num_; ++i) {
        vertical_component[segments_num * 2 * param_num_ + i] =
            first_order_coeff[i] * sgn.x();
        vertical_component[segments_num * 2 * param_num_ + param_num_ + i] =
            -first_order_coeff[i] * sgn.y();
    }
    inequ_constraint_.constraint_mat.emplace_back(direction_vec);
    inequ_constraint_.lower_bound.emplace_back(0);
    inequ_constraint_.upper_bound.emplace_back(
        std::numeric_limits<float>::max());
}  // namespace planning_lib

void SmoothQpSolver::AddSmoothConstraint(int32_t seg_counts) {
    if (seg_counts < 0 || seg_counts >= seg_num_) return;
    double noimal_s = 1;
    int32_t seg_param = 2 * param_num_;
    std::vector<double> smooth_constraint_x(opt_num_, 0);
    std::vector<double> smooth_constraint_y(opt_num_, 0);
    // 1 add point smooth
    for (int32_t i = 0; i < param_num_; ++i) {
        smooth_constraint_x[seg_counts * seg_param + i] = noimal_s;
        smooth_constraint_y[seg_counts * seg_param + param_num_ + i] = noimal_s;
    }
    smooth_constraint_x[(seg_counts + 1) * seg_param] = -1;
    smooth_constraint_y[(seg_counts + 1) * seg_param + param_num_] = -1;

    inequ_constraint_.constraint_mat.emplace_back(smooth_constraint_x);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);

    inequ_constraint_.constraint_mat.emplace_back(smooth_constraint_y);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);
    // 2 add derivate smooth
    std::vector<double> smooth_constraint_dx(opt_num_, 0);
    std::vector<double> smooth_constraint_dy(opt_num_, 0);
    for (int i = 1; i < param_num_; ++i) {
        smooth_constraint_dx[seg_counts * seg_param + i] = i * noimal_s;
        smooth_constraint_dy[seg_counts * seg_param + param_num_ + i] =
            i * noimal_s;
    }
    smooth_constraint_dx[(seg_counts + 1) * seg_param + 1] = -1;
    smooth_constraint_dy[(seg_counts + 1) * seg_param + param_num_ + 1] = -1;
    inequ_constraint_.constraint_mat.emplace_back(smooth_constraint_dx);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);

    inequ_constraint_.constraint_mat.emplace_back(smooth_constraint_dy);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);

    // 3 add second derivate smooth
    std::vector<double> smooth_constraint_ddx(opt_num_, 0);
    std::vector<double> smooth_constraint_ddy(opt_num_, 0);
    for (int i = 2; i < param_num_; ++i) {
        smooth_constraint_ddx[seg_counts * seg_param + i] =
            i * (i - 1) * noimal_s;
        smooth_constraint_ddy[seg_counts * seg_param + param_num_ + i] =
            i * (i - 1) * noimal_s;
    }
    smooth_constraint_ddx[(seg_counts + 1) * seg_param + 2] = -2;
    smooth_constraint_ddy[(seg_counts + 1) * seg_param + param_num_ + 2] = -2;
    inequ_constraint_.constraint_mat.emplace_back(smooth_constraint_ddx);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);

    inequ_constraint_.constraint_mat.emplace_back(smooth_constraint_ddy);
    inequ_constraint_.lower_bound.emplace_back(-epsilon_);
    inequ_constraint_.upper_bound.emplace_back(epsilon_);
}

void SmoothQpSolver::AddSmoothConstraint(void) {
    for (int32_t i = 0; i < seg_num_ - 1; ++i) {
        AddSmoothConstraint(i);
    }
}

void SmoothQpSolver::ConstraintClear() {
    inequ_constraint_.constraint_mat.clear();
    inequ_constraint_.lower_bound.clear();
    inequ_constraint_.upper_bound.clear();
}

}  // namespace planning_lib
}  // namespace jarvis