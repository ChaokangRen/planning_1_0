#pragma once
#include <vector>

#include "common.h"
namespace jarvis {
namespace planning_lib {
class DistcretizedPath {
public:
    DistcretizedPath() = default;
    explicit DistcretizedPath(const std::vector<PathPoint> &path_points);

    virtual ~DistcretizedPath() = default;

    void SetPathPoint(const std::vector<PathPoint> &path_points);

    double Length() const;

    const PathPoint &StartPoint() const;

    const PathPoint &EndPoint() const;

    PathPoint Evaluate(const double path_s) const;

    const std::vector<PathPoint> &PathPoints() const;

    int32_t NumOfPoints() const;

    virtual void Clear();

protected:
    std::vector<PathPoint>::const_iterator QueryLowerBound(
        const double path_s) const;

    std::vector<PathPoint> path_points_;
};
}  // namespace planning_lib
}  // namespace jarvis