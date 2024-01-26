#pragma once

#include <vector>

#include "common.h"
#include "lane_line.h"
#include "obstacle.h"
#include "polygon2d.h"
#include "quintic_polynomial_curve.h"
#include "sglog/sglog.h"
#include "sgtime/sgtime.h"

namespace jarvis {
namespace planning_lib {

class ReferenceLine {
public:
    ReferenceLine() = default;
    ReferenceLine(const ReferenceLine &refline) {
        curve_pairs_ = refline.curve_pairs_;
        lane_center_offset_cost_ = refline.lane_center_offset_cost_;
        center_lanes_ = refline.center_lanes_;
        lane_lines_ = refline.lane_lines_;
        size_ = refline.size_;
        single_curve_length_ = refline.single_curve_length_;
        path_length_ = refline.path_length_;
        time_length_ = refline.time_length_;
        ref_xys_ = refline.ref_xys_;
    }
    explicit ReferenceLine(
        std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
            &curve_pairs,
        double lane_center_offset_cost,
        const std::vector<std::vector<PathPoint>> &center_lanes,
        const double refline_total_length, const double refline_total_time)
        : curve_pairs_(curve_pairs),
          lane_center_offset_cost_(lane_center_offset_cost),
          center_lanes_(center_lanes) {
        size_ = curve_pairs_.size();
        single_curve_length_ = refline_total_length / size_;
        path_length_ = refline_total_length;
        time_length_ = refline_total_time;
        CalS2ref();
        for (int i = 0; i < center_lanes.size(); ++i) {
            lane_lines_.emplace_back(
                std::move(GetSlLineInFrenet(center_lanes[i])));
        }
    }

    explicit ReferenceLine(
        std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
            &curve_pairs,
        double lane_center_offset_cost, const double refline_total_length,
        const double refline_total_time);

    bool GetPathOptimizer(std::vector<Vec2d> *path);

    bool GetCartesianOptimalPath(const DpPathData &dp_path_data,
                                 std::vector<PathPoint> *path_opt) const;

    bool SLToXY(const SLPoint &sl_point, Vec2d *const xy_point) const;

    bool XyToSl(const double x, const double y, double *s, double *l) const;

    bool XyToSl(const double x, const double y, const double theta,
                double *pos_s, double *pos_l, double *delta_theta) const;

    bool SLToXyThetaKappa(const std::array<double, 3> &s_condition,
                          const std::array<double, 3> &d_condition, double *c_x,
                          double *c_y, double *c_theta, double *c_kappa,
                          double *c_v, double *c_a) const;

    double GetCurrentSHeading(const double s) const;

    double GetCurrentSKappa(const double s) const;

    LaneLine GetSlLineInFrenet(const std::vector<PathPoint> &path_line) const;

    void GetSlLineInFrenet1(const std::vector<PathPoint> &path_line,
                            LaneLine *lane_line);

    SLStaticBoundary GetSlBoundary(const Polygon2d &poly_obs) const;

    std::vector<LaneLine> CenterLines() const {
        return lane_lines_;
    }
    double LaneCenterOffsetCost() const {
        return lane_center_offset_cost_;
    }

    std::vector<SpeedPoint> GerReflineSpeedPoints(const int32_t id) const;

    void SetXYS(std::vector<std::pair<Vec2d, double>> ref_xys) {
        ref_xys_ = ref_xys;
        return;
    }
    void CalS2ref();

    double RefSpeed(void) const {
        return center_lanes_.front().front().dkappa;
    }

private:
    bool GetSFromXy(const double x, const double y, double *s, double *min_x,
                    double *min_y, double *theta) const;
    bool GetSFormXyBinarySearch(const double x, const double y, double *s,
                                double *min_x, double *min_y,
                                double *theta) const;
    double GetRefSpeed(const double s, const int32_t id) const;
    double PointToCurveDistSqr(const double x, const double y, const int32_t i,
                               const double s) const;

private:
    std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
        curve_pairs_;

    int32_t size_;

    double single_curve_length_;

    double sample_distance_s_ = 1;

    double path_length_ = 0.0;

    double time_length_ = 0.0;

    double safety_distance_ = 60;

    double lane_center_offset_cost_ = 0;

    std::vector<LaneLine> lane_lines_;

    std::vector<std::vector<PathPoint>> center_lanes_;

    std::vector<std::pair<Vec2d, double>> ref_xys_;
};
}  // namespace planning_lib
}  // namespace jarvis