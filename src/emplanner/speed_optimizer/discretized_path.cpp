#include "discretized_path.h"

#include <algorithm>
namespace jarvis {
namespace planning_lib {

PathPoint LinearInterpolateApproximation(const PathPoint &p0,
                                         const PathPoint &p1, const double s) {
    double s0 = p0.s_m;
    double s1 = p1.s_m;

    PathPoint path_point;
    double weight = (s - s0) / (s1 - s0);
    path_point.position_m.x =
        (1 - weight) * p0.position_m.x + weight * p1.position_m.x;
    path_point.position_m.y =
        (1 - weight) * p0.position_m.y + weight * p1.position_m.y;
    path_point.theta_rad =
        LinearInterpolateOfTheta(p0.theta_rad, p0.s_m, p1.theta_rad, p1.s_m, s);
    path_point.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
    path_point.dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
    path_point.ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;
    path_point.s_m = s;
    return path_point;
}
DistcretizedPath::DistcretizedPath(const std::vector<PathPoint> &path_points)
    : path_points_(path_points) {}
void DistcretizedPath::SetPathPoint(const std::vector<PathPoint> &path_points) {
    path_points_ = path_points;
}
double DistcretizedPath::Length() const {
    if (path_points_.empty()) {
        return 0.0;
    }
    return path_points_.back().s_m - path_points_.front().s_m;
}
PathPoint DistcretizedPath::Evaluate(const double path_s) const {
    std::vector<PathPoint>::const_iterator it_lower = QueryLowerBound(path_s);
    if (it_lower == path_points_.begin()) {
        return path_points_.front();
    }
    if (it_lower == path_points_.end()) {
        return path_points_.back();
    }
    return LinearInterpolateApproximation(*(it_lower - 1), *it_lower, path_s);
}
const std::vector<PathPoint> &DistcretizedPath::PathPoints() const {
    return path_points_;
}
int32_t DistcretizedPath::NumOfPoints() const {
    return path_points_.size();
}
const PathPoint &DistcretizedPath::StartPoint() const {
    return path_points_.front();
}
const PathPoint &DistcretizedPath::EndPoint() const {
    return path_points_.back();
}
void DistcretizedPath::Clear() {
    path_points_.clear();
}
std::vector<PathPoint>::const_iterator DistcretizedPath::QueryLowerBound(
    const double path_s) const {
    auto func = [](const PathPoint &tp, const double path_s) {
        return tp.s_m < path_s;
    };
    return std::lower_bound(path_points_.begin(), path_points_.end(), path_s,
                            func);
}
}  // namespace planning_lib
}  // namespace jarvis