#include <algorithm>

#include "common.h"
#include "sglog/sglog.h"
#include "st_boundary.h"
namespace jarvis {
namespace planning_lib {
double Vec3CrossPord(const Vec2d &pos1, const Vec2d &pos2, const Vec2d &pos3) {
    Vec2d vec1(pos2.x() - pos1.x(), pos2.y() - pos1.y());
    Vec2d vec2(pos3.x() - pos1.x(), pos3.y() - pos1.y());
    return vec1.CrossProd(vec2);
}
StBoundary::StBoundary(
    const std::vector<std::pair<STPoint, STPoint>> &point_paris) {
    std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_paris);
    RemoveRedundantPoints(&reduced_pairs);

    for (const std::pair<STPoint, STPoint> &item : reduced_pairs) {
        const double t = item.first.t();
        lower_points_.emplace_back(item.first.s(), t);
        upper_points_.emplace_back(item.second.s(), t);
        // SG_INFO("t = %lf,lo = %lf,up = %lf", t, item.first.s(),
        //         item.second.s());
    }
    for (std::vector<STPoint>::iterator it = lower_points_.begin();
         it != lower_points_.end(); ++it) {
        points_.emplace_back(it->x(), it->y());
    }
    for (std::vector<STPoint>::reverse_iterator rit = upper_points_.rbegin();
         rit != upper_points_.rend(); ++rit) {
        points_.emplace_back(rit->x(), rit->y());
    }
    BuildFromPoints();
    for (const STPoint &point : lower_points_) {
        min_s_ = std::fmin(min_s_, point.s());
    }
    for (const STPoint &point : upper_points_) {
        max_s_ = std::fmax(max_s_, point.s());
    }
    min_t_ = lower_points_.front().t();
    max_t_ = lower_points_.back().t();
}
bool StBoundary::IsPointInBoundary(const STPoint &st_point) const {
    if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
        return false;
    }
    int32_t left = 0;
    int32_t right = 0;
    if (!GetIndexRange(lower_points_, st_point.t(), &left, &right)) {
        SG_ERROR("fail to get index range");
        return false;
    }
    const double check_upper =
        Vec3CrossPord(st_point, upper_points_[left], upper_points_[right]);
    const double check_lower =
        Vec3CrossPord(st_point, lower_points_[left], lower_points_[right]);
    return (check_upper * check_lower < 0);
}
bool StBoundary::GetIndexRange(const std::vector<STPoint> &points,
                               const double t, int32_t *left,
                               int32_t *right) const {
    if (t < points.front().t() || t > points.back().t()) {
        SG_ERROR("t is out of range. t = %f ", t);
        return false;
    }
    auto comp = [](const STPoint &p, const double t) { return p.t() < t; };

    std::vector<STPoint>::const_iterator first_point =
        std::lower_bound(points.begin(), points.end(), t, comp);

    int32_t index = std::distance(points.begin(), first_point);
    if (index == 0) {
        *left = *right = 0;
    } else if (first_point == points.end()) {
        *left = *right = points.size() - 1;
    } else {
        *left = index - 1;
        *right = index;
    }
    return true;
}

STPoint StBoundary::BottomLeftPoint() const {
    if (lower_points_.empty()) {
        SG_ERROR("StBoundary has zero points.");
    }
    return lower_points_.front();
}
STPoint StBoundary::BottomRightPoint() const {
    if (lower_points_.empty()) {
        SG_ERROR("StBoundary has zero points.");
    }
    return lower_points_.front();
}
StBoundary StBoundary::ExpandByS(const double s) const {
    if (lower_points_.empty()) {
        return StBoundary();
    }
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    for (int32_t i = 0; i < lower_points_.size(); ++i) {
        point_pairs.emplace_back(
            STPoint(lower_points_[i].s() - s, lower_points_[i].t()),
            STPoint(upper_points_[i].s() - s, upper_points_[i].t()));
    }
    return StBoundary(std::move(point_pairs));
}
StBoundary StBoundary::ExpandByT(const double t) const {
    if (lower_points_.empty()) {
        return StBoundary();
    }
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    // 1 Use linear interpolation at the front
    double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
    double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
    double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

    point_pairs.emplace_back(
        STPoint(lower_points_[0].s() - t * lower_left_delta_s / left_delta_t,
                lower_points_[0].t() - t),
        STPoint(upper_points_[0].s() - t * lower_left_delta_s / left_delta_t,
                upper_points_[0].t() - t));

    double min_epsilon = 1e-3;
    point_pairs.front().first.set_s(
        std::fmin(point_pairs.front().second.s() - min_epsilon,
                  point_pairs.front().first.s()));
    // 2 Sequential interpolation
    for (int32_t i = 0; i < lower_points_.size(); ++i) {
        point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
    }

    // 3 Use linear interpolation at the back
    int32_t length = lower_points_.size();

    const double right_delta_t =
        lower_points_[length - 1].t() - lower_points_[length - 2].t();
    const double lower_right_delta_s =
        lower_points_[length - 1].s() - lower_points_[length - 2].s();
    const double upper_right_delta_s =
        upper_points_[length - 1].s() - upper_points_[length - 2].s();

    point_pairs.emplace_back(
        STPoint(
            lower_points_.back().s() + t * lower_right_delta_s / right_delta_t,
            lower_points_.back().t() + t),
        STPoint(
            upper_points_.back().s() + t * lower_right_delta_s / right_delta_t,
            upper_points_.back().t() + t));
    point_pairs.back().second.set_s(
        std::fmax(point_pairs.back().second.s(),
                  point_pairs.back().first.s() + min_epsilon));

    return StBoundary(std::move(point_pairs));
}
bool StBoundary::IsPointNear(const LineSegment2d &seg, const Vec2d &point,
                             const double max_dist) {
    return seg.DistanceTo(point) < max_dist;
}
void StBoundary::RemoveRedundantPoints(
    std::vector<std::pair<STPoint, STPoint>> *point_pairs) {
    double max_dist = 0.1;
    int32_t i = 0;
    int32_t j = 1;

    for (; j + 1 < point_pairs->size() && i < point_pairs->size(); ++j) {
        LineSegment2d lower_seg(point_pairs->at(i).first,
                                point_pairs->at(j + 1).first);
        LineSegment2d upper_seg(point_pairs->at(i).second,
                                point_pairs->at(j + 1).second);
        if (!IsPointNear(lower_seg, point_pairs->at(j).first, max_dist) ||
            !IsPointNear(upper_seg, point_pairs->at(j).first, max_dist)) {
            ++i;
            if (i != j) {
                point_pairs->at(i) = point_pairs->at(j);
            }
        }
    }
    point_pairs->at(++i) = point_pairs->back();
    point_pairs->resize(i + 1);
}
const std::string &StBoundary::id() const {
    return id_;
}
void StBoundary::SetId(const std::string &id) {
    id_ = id;
}

bool StBoundary::GetUnblockSRange(const double curr_time, double *s_upper,
                                  double *s_lower) const {
    *s_upper = s_up_limit_;
    *s_lower = 0.0;
    if (curr_time < min_t_ || curr_time > max_t_) {
        return true;
    }
    int32_t left = 0;
    int32_t right = 0;
    if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
        SG_ERROR(" Fail to get index range");
        return false;
    }
    // linear interpolation
    double r = (curr_time - upper_points_[left].t()) /
               (upper_points_.at(right).t() - upper_points_.at(left).t());
    double upper_cross_s =
        upper_points_[left].s() +
        r * (upper_points_[right].s() - upper_points_[left].s());
    double lower_cross_s =
        lower_points_[left].s() +
        r * (lower_points_[right].s() - lower_points_[left].s());

    if (boundary_type_ == BoundaryType::STOP ||
        boundary_type_ == BoundaryType::YIELD ||
        boundary_type_ == BoundaryType::FOLLOW) {
        *s_upper = lower_cross_s;
    } else if (boundary_type_ == BoundaryType::OVERTAKE) {
        *s_lower = std::fmax(*s_lower, upper_cross_s);
    } else {
        return false;
    }
    return true;
}
bool StBoundary::GetBoundarySRange(const double curr_time, double *s_upper,
                                   double *s_lower) const {
    if (curr_time < min_t_ || curr_time > max_t_) {
        return false;
    }
    int32_t left = 0;
    int32_t right = 0;
    if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
        SG_ERROR("Fail to get index range");
        return false;
    }
    double r = (curr_time - upper_points_[left].t()) /
               (upper_points_.at(right).t() - upper_points_.at(left).t());

    *s_upper = upper_points_[left].s() +
               r * (upper_points_[right].s() - upper_points_[left].s());

    *s_lower = lower_points_[left].s() +
               r * (lower_points_[right].s() - lower_points_[left].s());

    // SG_INFO("lower = %lf,upper = %lf,sl = %lf,su = %lf",
    //         lower_points_[left].s(), upper_points_[left].s(), *s_lower,
    //         *s_upper);

    *s_upper = std::fmin(*s_upper, s_up_limit_);
    *s_lower = std::fmax(*s_lower, 0.0);
    return true;
}
double StBoundary::MinS() const {
    return min_s_;
}
double StBoundary::MinT() const {
    return min_t_;
}
double StBoundary::MaxS() const {
    return max_s_;
}
double StBoundary::MaxT() const {
    return max_t_;
}

std::vector<STPoint> StBoundary::upper_points() const {
    return upper_points_;
}
std::vector<STPoint> StBoundary::lower_points() const {
    return lower_points_;
}
StBoundary::BoundaryType StBoundary::boundary_type() const {
    return boundary_type_;
}
void StBoundary::SetBoundaryType(const BoundaryType &boundary_type) {
    boundary_type_ = boundary_type;
}
StBoundary StBoundary::GenerateStBoundary(
    const std::vector<STPoint> &lower_points,
    const std::vector<STPoint> &upper_points) {
    if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
        return StBoundary();
    }
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    for (int32_t i = 0; i < lower_points.size() && i < lower_points.size();
         ++i) {
        point_pairs.emplace_back(
            STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
            STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
    }
    return StBoundary(point_pairs);
}
StBoundary StBoundary::CutOffByT(const double t) const {
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    for (int32_t i = 0; i < lower_points_.size() && i < upper_points_.size();
         ++i) {
        if (lower_points[i].t() < t) {
            continue;
        }
        lower_points.push_back(lower_points_[i]);
        upper_points.push_back(upper_points_[i]);
    }
    return GenerateStBoundary(lower_points, upper_points);
}

}  // namespace planning_lib
}  // namespace jarvis