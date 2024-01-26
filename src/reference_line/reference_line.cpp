#include <Eigen/Eigen>
#include <cmath>

#include "cartesian_frenet_conversion.h"
#include "reference_line.h"
#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

ReferenceLine::ReferenceLine(
    std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
        &curve_pairs,
    double lane_center_offset_cost, const double refline_total_length,
    const double refline_total_time)
    : curve_pairs_(curve_pairs),
      lane_center_offset_cost_(lane_center_offset_cost) {
    size_ = curve_pairs_.size();
    single_curve_length_ = refline_total_length / size_;
    path_length_ = refline_total_length;
    time_length_ = refline_total_time;
}

bool ReferenceLine::XyToSl(const double x, const double y, double *pos_s,
                           double *pos_l) const {
    double ref_x = 0;
    double ref_y = 0;
    double s = 0;
    double ref_theta = 0;
    std::array<double, 3> frenet_s;
    std::array<double, 3> frenet_l;
    GetSFromXy(x, y, &s, &ref_x, &ref_y, &ref_theta);

    CartesianFrenetConverter::CartesianToFrenet(s, ref_x, ref_y, ref_theta, x,
                                                y, pos_s, pos_l);
    return true;
}
bool ReferenceLine::XyToSl(const double x, const double y, const double theta,
                           double *pos_s, double *pos_l,
                           double *delta_theta) const {
    double ref_x = 0;
    double ref_y = 0;
    double s = 0;
    double ref_theta = 0;
    std::array<double, 3> frenet_s;
    std::array<double, 3> frenet_l;
    GetSFromXy(x, y, &s, &ref_x, &ref_y, &ref_theta);

    (*delta_theta) = ref_theta - theta;

    CartesianFrenetConverter::CartesianToFrenet(s, ref_x, ref_y, ref_theta, x,
                                                y, pos_s, pos_l);
    return true;
}

void ReferenceLine::CalS2ref() {
    double epsilon_s = sample_distance_s_ / 1000;
    double min_dist = std::numeric_limits<float>::infinity();
    std::vector<std::pair<Vec2d, double>> ref_xys;
    for (int32_t i = 0; i < size_; ++i) {
        for (double delta_s = -0.1; delta_s < sample_distance_s_;
             delta_s += epsilon_s) {
            double ref_x = curve_pairs_[i].first.CalcuteCurveValue(delta_s);
            double ref_y = curve_pairs_[i].second.CalcuteCurveValue(delta_s);
            double s = (delta_s + i) * single_curve_length_;
            Vec2d point(ref_x, ref_y);
            std::pair<Vec2d, double> point_s(point, s);
            ref_xys.emplace_back(point_s);
        }
    }
    SetXYS(ref_xys);
    return;
}

bool ReferenceLine::GetSFromXy(const double x, const double y, double *s,
                               double *min_x, double *min_y,
                               double *theta) const {
    // SG_WARN("ref_xys=%d", ref_xys_.size());
    int start = 1;
    int end = ref_xys_.size();
    bool behind_first_point = false;

    double ref_x_s = ref_xys_[start].first.x();
    double ref_y_s = ref_xys_[start].first.y();
    double pre_ref_x_s = ref_xys_[start - 1].first.x();
    double pre_ref_y_s = ref_xys_[start - 1].first.y();
    double tmp_theta_s =
        Vec2d(ref_x_s - pre_ref_x_s, ref_y_s - pre_ref_y_s).Angle();
    while (start < end) {
        // SG_INFO("end=%d,start=%d", end, start);
        int mid = std::floor((start + end) / 2);
        ref_x_s = ref_xys_[start].first.x();
        ref_y_s = ref_xys_[start].first.y();
        pre_ref_x_s = ref_xys_[start - 1].first.x();
        pre_ref_y_s = ref_xys_[start - 1].first.y();
        tmp_theta_s =
            Vec2d(ref_x_s - pre_ref_x_s, ref_y_s - pre_ref_y_s).Angle();

        double ref_x_e = ref_xys_[mid].first.x();
        double ref_y_e = ref_xys_[mid].first.y();
        double pre_ref_x_e = ref_xys_[mid - 1].first.x();
        double pre_ref_y_e = ref_xys_[mid - 1].first.y();
        double tmp_theta_e =
            Vec2d(ref_x_e - pre_ref_x_e, ref_y_e - pre_ref_y_e).Angle();

        Vec2d x2projs(x - ref_x_s, y - ref_y_s);
        Vec2d x2proje(x - ref_x_e, y - ref_y_e);
        Vec2d unitv_s(std::cos(tmp_theta_s), std::sin(tmp_theta_s));
        Vec2d unitv_e(std::cos(tmp_theta_e), std::sin(tmp_theta_e));
        double dir_s = x2projs.InnerProd(unitv_s);
        double dir_e = x2proje.InnerProd(unitv_e);
        if (dir_s < 0 && start == 1) {
            behind_first_point = true;
            break;
        }
        if (dir_e < 0) {
            end = mid;
        } else {
            start = mid + 1;
        }
    }
    int index = std::fmax(0, start - 5);
    double min_dist = std::numeric_limits<float>::infinity();
    double min_index;
    while (index < std::fmin(ref_xys_.size(), start + 5)) {
        ref_x_s = ref_xys_[index].first.x();
        ref_y_s = ref_xys_[index].first.y();
        double dist =
            (ref_x_s - x) * (ref_x_s - x) + (ref_y_s - y) * (ref_y_s - y);
        dist = sqrt(dist);
        if (dist < min_dist) {
            min_dist = dist;
            min_index = index;
        }
        index++;
    }

    double ref_theta = NormalizeAngle(tmp_theta_s);
    Vec2d l(std::cos(ref_theta), std::sin(ref_theta));
    Vec2d r((x - ref_x_s), (y - ref_y_s));
    double delta_s = l.InnerProd(r);

    // s += delta_s;
    double delta_x = delta_s * cos(ref_theta);
    double delta_y = delta_s * sin(ref_theta);
    if (behind_first_point) {
        delta_s = 0;
        delta_x = 0;
        delta_y = 0;
    }
    // SG_INFO("delta_s = %f", delta_s);
    // double proj_x = ref_x + delta_x;
    // double proj_y = ref_y + delta_y;

    // *s = ref_xys_[start].second + delta_s;
    // *min_x = ref_x_s + delta_x;
    // *min_y = ref_y_s + delta_y;
    *s = ref_xys_[min_index].second;
    *min_x = ref_xys_[min_index].first.x();
    *min_y = ref_xys_[min_index].first.y();
    *theta = tmp_theta_s;
    // SG_INFO("s=%f,min_x=%f", *s, *min_x);
    return true;
}
// bool ReferenceLine::GetSFormXy(const double x, const double y, double *s,
//                                double *min_x, double *min_y,
//                                double *theta) const {
//     double epsilon_s = sample_distance_s_ / 100;
//     double min_dist = std::numeric_limits<float>::infinity();
//     for (int32_t i = 0; i < size_; ++i) {
//         for (double delta_s = -0.5; delta_s < sample_distance_s_;
//              delta_s += epsilon_s) {
//             double ref_x = curve_pairs_[i].first.CalcuteCurveValue(delta_s);
//             double ref_y = curve_pairs_[i].second.CalcuteCurveValue(delta_s);
//             double dist = (ref_x - x) * (ref_x - x) + (ref_y - y) * (ref_y -
//             y); dist = sqrt(dist); if (dist < min_dist) {
//                 min_dist = dist;
//                 *s = (delta_s + i) * single_curve_length_;
//                 *min_x = ref_x;
//                 *min_y = ref_y;
//                 double ref_x_pre =
//                     curve_pairs_[i].first.CalcuteCurveValue(delta_s - 0.05);
//                 double ref_y_pre =
//                     curve_pairs_[i].second.CalcuteCurveValue(delta_s - 0.05);
//                 *theta = Vec2d(ref_x - ref_x_pre, ref_y - ref_y_pre).Angle();
//             }
//         }
//     }
//     return true;
// }

double ReferenceLine::PointToCurveDistSqr(const double x, const double y,
                                          const int32_t i,
                                          const double s) const {
    double dx = x - curve_pairs_[i].first.CalcuteCurveValue(s);
    double dy = y - curve_pairs_[i].second.CalcuteCurveValue(s);
    return dx * dx + dy * dy;
}
bool ReferenceLine::GetSFormXyBinarySearch(const double x, const double y,
                                           double *s, double *min_x,
                                           double *min_y, double *theta) const {
    double start = 0;
    double end = sample_distance_s_ * size_ - 0.001;
    double middle = (end - start) / 2;
    // while (end - start > 0.01) {
    //     int32_t left_i = std::floor(start / sample_distance_s_);
    //     double left_s = std::fmod(start, sample_distance_s_);
    //     int32_t middle_i = std::floor(middle / sample_distance_s_);
    //     double middle_s = std::fmod(middle, sample_distance_s_);
    //     int32_t right_i = std::floor(end / sample_distance_s_);
    //     double right_s = std::fmod(end, sample_distance_s_);

    //     double left_dist = PointToCurveDistSqr(x, y, left_i, left_s);
    //     double middle_dist = PointToCurveDistSqr(x, y, middle_i, middle_s);
    //     double right_dist = PointToCurveDistSqr(x, y, right_i, right_s);
    // }
    return true;
}

bool ReferenceLine::GetCartesianOptimalPath(
    const DpPathData &dp_path_data, std::vector<PathPoint> *path_opt) const {
    if (dp_path_data.size() <= 0) {
        SG_ERROR("reference_line:path length error!");
        return false;
    }
    for (int32_t i = 0; i < dp_path_data.size(); ++i) {
        double path_s = dp_path_data[i].pos.s;
        double path_l = dp_path_data[i].pos.l;

        double segments_length = path_length_ / curve_pairs_.size();
        int32_t seg_num = std::floor(path_s / segments_length);
        double curr_s = std::fmod(path_s, segments_length) / segments_length;

        if (seg_num >= curve_pairs_.size()) {
            curr_s = (curve_pairs_.size() - seg_num + 1) + curr_s;
            seg_num = curve_pairs_.size() - 1;
        }
        double ref_x = curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s);
        double ref_y = curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s);
        double ref_x_pre =
            curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s + 0.5);
        double ref_y_pre =
            curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s + 0.5);
        static double theta = 0;
        theta = Vec2d(ref_x_pre - ref_x, ref_y_pre - ref_y).Angle();
        double curr_x = 0;
        double curr_y = 0;
        CartesianFrenetConverter::FrentToCartesian(ref_x, ref_y, theta, path_s,
                                                   path_l, &curr_x, &curr_y);

        PathPoint path_point{curr_x, curr_y, 0, theta, 0, path_s, 0, 0};
        path_opt->emplace_back(path_point);
    }
    for (int32_t i = 1; i < path_opt->size(); ++i) {
        if (path_opt->size() <= 1) {
            (*path_opt)[0].theta_rad = 0;
            return false;
        } else {
            (*path_opt)[i].theta_rad = Vec2d((*path_opt)[i + 1].position_m.x -
                                                 (*path_opt)[i].position_m.x,
                                             (*path_opt)[i + 1].position_m.y -
                                                 (*path_opt)[i].position_m.y)
                                           .Angle();
        }
    }
    (*path_opt)[0].theta_rad = (*path_opt)[1].theta_rad;
    int32_t step_cnt = 40;
    for (int32_t j = 0; j < path_opt->size(); ++j) {
        // int i = path_opt->size() / 2;
        // int k = path_opt->size() - 1 - j;
        int32_t k =
            std::min(step_cnt + j, static_cast<int32_t>(path_opt->size() - 1));
        int32_t i = (k - j) / 2 + j;
        double ds_a = (*path_opt)[i].s_m - (*path_opt)[j].s_m;
        double ds_b = (*path_opt)[k].s_m - (*path_opt)[i].s_m;
        Eigen::Vector3d x_vec;
        x_vec << (*path_opt)[j].position_m.x, (*path_opt)[i].position_m.x,
            (*path_opt)[k].position_m.x;
        Eigen::Vector3d y_vec;

        y_vec << (*path_opt)[j].position_m.y, (*path_opt)[i].position_m.y,
            (*path_opt)[k].position_m.y;
        Eigen::Matrix3d mat;
        mat << 1, -ds_a, ds_a * ds_a, 1, 0, 0, 1, ds_b, ds_b * ds_b;
        Eigen::Vector3d a_vec = mat.inverse() * x_vec;
        Eigen::Vector3d b_vec = mat.inverse() * y_vec;
        double kappa =
            2 * (a_vec(2) * b_vec(1) - a_vec(1) * b_vec(2)) /
            std::pow((a_vec(1) * a_vec(1) + b_vec(1) * b_vec(1)), 1.5);
        (*path_opt)[j].kappa = kappa;
        double ref_vel = GetRefSpeed((*path_opt)[j].s_m, 0);
        (*path_opt)[j].dkappa = ref_vel;
    }
    (*path_opt)[0].kappa = (*path_opt)[1].kappa;
    (*path_opt)[0].dkappa = (*path_opt)[1].dkappa;
    int32_t back_count = path_opt->size() - 1;
    (*path_opt)[back_count].kappa = (*path_opt)[back_count - 1].kappa;
    (*path_opt)[back_count].dkappa = (*path_opt)[back_count - 1].dkappa;
    return true;
}
double ReferenceLine::GetRefSpeed(const double s, const int32_t id) const {
    if (s < 0) {
        return center_lanes_[id].front().dkappa;
    }
    for (int32_t i = 0; i < center_lanes_[id].size(); ++i) {
        if (center_lanes_[id][i].s_m > s) {
            return center_lanes_[id][i].dkappa;
        }
    }
    return center_lanes_[id].back().dkappa;
}
bool ReferenceLine::SLToXY(const SLPoint &sl_point,
                           Vec2d *const xy_point) const {
    if (xy_point == nullptr) return false;

    double segments_length = path_length_ / curve_pairs_.size();
    int32_t seg_num = std::floor(sl_point.s / segments_length);

    int32_t seg_num_over = curve_pairs_.size() - seg_num;

    double curr_s = std::fmod(sl_point.s, segments_length) / segments_length;

    if (seg_num_over <= 0) {
        seg_num = curve_pairs_.size() - 1;
        curr_s = sl_point.s - path_length_;
    }

    double ref_x = curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s);

    double ref_y = curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s);
    double ref_x_pre =
        curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s - 0.1);
    double ref_y_pre =
        curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s - 0.1);

    double theta = Vec2d(ref_x - ref_x_pre, ref_y - ref_y_pre).Angle();
    double curr_x = 0;
    double curr_y = 0;

    double tmp_x = 0;
    double tmp_y = 0;

    CartesianFrenetConverter::FrentToCartesian(ref_x, ref_y, theta, sl_point.s,
                                               sl_point.l, &tmp_x, &tmp_y);
    // SG_INFO(
    //     "segnum = %d,currs = %lf,s = %lf,l = %lf,ref_x = %lf,ref_y = %lf,tmpx
    //     "
    //     "= %lf,tmpy = %lf",
    //     seg_num, curr_s, sl_point.s, sl_point.l, ref_x, ref_y, tmp_x, tmp_y);
    xy_point->set_x(tmp_x);
    xy_point->set_y(tmp_y);
    return true;
}

bool ReferenceLine::SLToXyThetaKappa(const std::array<double, 3> &s_condition,
                                     const std::array<double, 3> &d_condition,
                                     double *c_x, double *c_y, double *c_theta,
                                     double *c_kappa, double *c_v,
                                     double *c_a) const {
    double segments_length = path_length_ / curve_pairs_.size();
    int32_t seg_num = std::floor(s_condition[0] / segments_length);
    double curr_s =
        std::fmod(s_condition[0], segments_length) / segments_length;

    double ref_x = curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s);
    double ref_y = curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s);
    double ref_x_pre =
        curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s - 0.1);
    double ref_y_pre =
        curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s - 0.1);

    double ref_theta = Vec2d(ref_x - ref_x_pre, ref_y - ref_y_pre).Angle();
    double ref_s = s_condition[0];

    double ref_dx =
        curve_pairs_[seg_num].first.CalcuteDerivativeCurveValue(curr_s);
    double ref_dy =
        curve_pairs_[seg_num].second.CalcuteDerivativeCurveValue(curr_s);
    double ref_ddx =
        curve_pairs_[seg_num].first.CalcuteSecDerivativeCurveValue(curr_s);
    double ref_ddy =
        curve_pairs_[seg_num].second.CalcuteSecDerivativeCurveValue(curr_s);
    double ref_dddx =
        curve_pairs_[seg_num].first.CalcuteThdDerivativeCurveValue(curr_s);
    double ref_dddy =
        curve_pairs_[seg_num].second.CalcuteThdDerivativeCurveValue(curr_s);
    double ref_kappa = (ref_dx * ref_ddy - ref_ddx * ref_dy) /
                       std::pow((ref_dx * ref_dx + ref_dy * ref_dy), 1.5);
    double ref_dkappa = ((ref_dddy * ref_dx - ref_dddx * ref_dy) *
                             (ref_dx * ref_dx + ref_dy * ref_dy) -
                         3 * (ref_dx * ref_ddx + ref_dy * ref_ddy) *
                             (ref_ddy * ref_dx - ref_ddx * ref_dy)) /
                        std::pow(ref_dx * ref_dx + ref_dy * ref_dy, 3);

    CartesianFrenetConverter::FrenetToCartesian(
        ref_s, ref_x, ref_y, ref_theta, ref_kappa, ref_dkappa, s_condition,
        d_condition, c_x, c_y, c_theta, c_kappa, c_v, c_a);
    return false;
}

double ReferenceLine::GetCurrentSHeading(const double s) const {
    double segments_length = path_length_ / curve_pairs_.size();
    int32_t seg_num = std::floor(s / segments_length);
    double curr_s = std::fmod(s, segments_length) / segments_length;

    double ref_x = curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s);
    double ref_y = curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s);
    double ref_x_pre =
        curve_pairs_[seg_num].first.CalcuteCurveValue(curr_s - 0.1);
    double ref_y_pre =
        curve_pairs_[seg_num].second.CalcuteCurveValue(curr_s - 0.1);

    double theta = Vec2d(ref_x - ref_x_pre, ref_y - ref_y_pre).Angle();
    return theta;
}

double ReferenceLine::GetCurrentSKappa(const double s) const {
    double segments_length = path_length_ / curve_pairs_.size();
    int32_t seg_num = std::floor(s / segments_length);
    double curr_s = std::fmod(s, segments_length) / segments_length;

    double dot_x =
        curve_pairs_[seg_num].first.CalcuteDerivativeCurveValue(curr_s);
    double dot_y =
        curve_pairs_[seg_num].second.CalcuteDerivativeCurveValue(curr_s);

    double ddot_x =
        curve_pairs_[seg_num].first.CalcuteSecDerivativeCurveValue(curr_s);
    double ddot_y =
        curve_pairs_[seg_num].second.CalcuteSecDerivativeCurveValue(curr_s);

    return abs(dot_x * ddot_y - ddot_x * dot_y) /
           std::pow(dot_x * dot_x + dot_y * dot_y, 1.5);
}

LaneLine ReferenceLine::GetSlLineInFrenet(
    const std::vector<PathPoint> &path_line) const {
    std::vector<SLPoint> sl_path_line(path_line.size(), SLPoint());

    double t1 = SgTimeUtil::now_secs();
    for (int32_t i = 0; i < path_line.size(); ++i) {
        XyToSl(path_line[i].position_m.x, path_line[i].position_m.y,
               &sl_path_line[i].s, &sl_path_line[i].l);
    }
    double t2 = SgTimeUtil::now_secs();

    return LaneLine(std::move(sl_path_line));
}

SLStaticBoundary ReferenceLine::GetSlBoundary(const Polygon2d &poly_obs) const {
    std::vector<SLPoint> sl_points;
    for (const Vec2d &point : poly_obs.points()) {
        double s = 0;
        double l = 0;
        XyToSl(point.x(), point.y(), &s, &l);
        double ref_x = 0;
        double ref_y = 0;
        double sx = 0;
        double ref_theta = 0;

        GetSFromXy(point.x(), point.y(), &sx, &ref_x, &ref_y, &ref_theta);
        sl_points.emplace_back(SLPoint{s, l});
    }
    double start_s = std::numeric_limits<float>::infinity();
    double end_s = -std::numeric_limits<float>::infinity();
    double start_l = std::numeric_limits<float>::infinity();
    double end_l = -std::numeric_limits<float>::infinity();
    for (const SLPoint &point : sl_points) {
        start_s = std::min(start_s, point.s);
        end_s = std::max(end_s, point.s);
        start_l = std::min(start_l, point.l);
        end_l = std::max(end_l, point.l);
    }

    return SLStaticBoundary{start_s, end_s, start_l, end_l};
}
std::vector<SpeedPoint> ReferenceLine::GerReflineSpeedPoints(
    const int32_t id) const {
    std::vector<SpeedPoint> speed_points;
    double trajectory_time_interval = 0.01;
    for (float t = 0; t < time_length_; t += trajectory_time_interval) {
        SpeedPoint point;
        point.t = t;
        // point.velocity = 10;
        if (speed_points.empty() == true) {
            point.velocity = GetRefSpeed(0, id);
        } else {
            point.velocity = GetRefSpeed(speed_points.back().s, id);
        }

        point.s = point.velocity * t;
        point.acc = 0;
        point.dot_acc = 0;
        speed_points.emplace_back(point);
    }
    return speed_points;
}
}  // namespace planning_lib
}  // namespace jarvis