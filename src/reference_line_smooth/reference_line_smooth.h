#pragma once
#include <vector>

#include "common.h"
#include "quintic_polynomial_curve.h"
#include "smooth_qp_solver.h"
#include "vec2d.h"

namespace jarvis {
namespace planning_lib {
class ReferenceLineSmooth {
public:
    ReferenceLineSmooth();

    bool Init(const ReferenceLineSmoothConf &reference_line_smooth_conf);

    std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
    Smooth(const std::vector<PathPoint> &center_laneline,
           const PosFromIns &pos_ins);

    double ReferenceLineCost() {
        return reference_line_cost_;
    }

    std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
    CurvePairs() {
        return curve_pairs_;
    }

private:
    void CaculateReferenceLineCost(
        const std::vector<PathPoint> &center_laneline);

private:
    std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
        curve_pairs_;

    const std::vector<Point3d> *local_path_;

    std::vector<std::vector<double>> global_path_;

    SmoothQpSolver smooth_qp_solver_;

    std::vector<Vec2d> local_left_lane_line_;

    std::vector<Vec2d> local_right_lane_line_;

    float ref_line_length_ = 0.0;
    int32_t local_path_point_num_ = 0;
    int32_t segments_num_ = 0;
    float discrete_point_interval_ = 0.0;
    float segment_length_ = 0.0;
    double ref_line_cost_weight_ = 0.0;

    double reference_line_cost_ = 0.0;
};
}  // namespace planning_lib
}  // namespace jarvis
