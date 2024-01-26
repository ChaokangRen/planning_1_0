#include "reference_line_smooth.h"

#include <jsoncpp/json/json.h>

#include <fstream>
#include <iostream>

#include "cartesian_frenet_conversion.h"
#include "quintic_polynomial_curve.h"
#include "sglog/sglog.h"
#include "sgtime/sgtime.h"

namespace jarvis {
namespace planning_lib {

bool ReferenceLineSmooth::Init(
    const ReferenceLineSmoothConf &reference_line_smooth_conf) {
    SG_INFO("ReferenceLineSmooth Init");

    ref_line_length_ = reference_line_smooth_conf.refline_total_length;
    local_path_point_num_ = reference_line_smooth_conf.refline_total_num;
    segments_num_ = reference_line_smooth_conf.seg_num;
    discrete_point_interval_ = reference_line_smooth_conf.interval;
    segment_length_ = ref_line_length_ / segments_num_;
    ref_line_cost_weight_ = reference_line_smooth_conf.ref_line_cost_weight;

    smooth_qp_solver_.Init(reference_line_smooth_conf);

    return true;
}

ReferenceLineSmooth::ReferenceLineSmooth() {
    SG_INFO("ReferenceLineSmooth constructor");
}

std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
ReferenceLineSmooth::Smooth(const std::vector<PathPoint> &center_laneline,
                            const PosFromIns &pos_ins) {
    curve_pairs_.clear();
    smooth_qp_solver_.QpSolver(center_laneline, pos_ins, &curve_pairs_);
    CaculateReferenceLineCost(center_laneline);
    return curve_pairs_;
}

void ReferenceLineSmooth::CaculateReferenceLineCost(
    const std::vector<PathPoint> &center_laneline) {
    reference_line_cost_ = 0;
    for (int i = 0; i < center_laneline.size(); ++i) {
        double s = center_laneline[i].s_m;
        double segments_length = ref_line_length_ / curve_pairs_.size();
        int32_t seg_num = std::floor(s / segments_length);
        double curr_s = std::fmod(s, segments_length) / segments_length;
        if (seg_num >= curve_pairs_.size()) {
            break;
        }
        double dx = curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s) -
                    center_laneline[i].position_m.x;
        double dy = curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s) -
                    center_laneline[i].position_m.y;

        reference_line_cost_ += (dx * dx + dy * dy) * reference_line_cost_;
    }
}

}  // namespace planning_lib
}  // namespace jarvis