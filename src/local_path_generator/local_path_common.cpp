#include "local_path_common.h"
#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {
void SearchNearestCurve(const PosFromIns &vehicle_state,
                        const std::vector<Point3d> &left_curvature_line,
                        const std::vector<Point3d> &lateral_extended_line,
                        std::vector<Point3d> &curvature_line) {
    double dist_min_sqr = 1e10;
    int32_t count_min = -1;
    for (int32_t i = 0; i < left_curvature_line.size(); ++i) {
        double dx = left_curvature_line[i].x - vehicle_state.position.x;
        double dy = left_curvature_line[i].y - vehicle_state.position.y;
        double dist_sqr = dx * dx + dy * dy;
        if (dist_sqr < dist_min_sqr) {
            dist_min_sqr = dist_sqr;
            count_min = i;
        }
    }
    if (count_min == -1) {
        SG_ERROR("curvature generator count_min error");
    }
    if (count_min < left_curvature_line.size() - 1) {
        for (int32_t i = count_min; i < left_curvature_line.size(); ++i) {
            curvature_line.emplace_back(left_curvature_line[i]);
        }
        for (int32_t i = 0; i < lateral_extended_line.size(); ++i) {
            curvature_line.emplace_back(lateral_extended_line[i]);
        }
    } else {
        dist_min_sqr = 1e10;
        count_min = -1;
        for (int32_t i = 0; i < lateral_extended_line.size(); ++i) {
            double dx = lateral_extended_line[i].x - vehicle_state.position.x;
            double dy = lateral_extended_line[i].y - vehicle_state.position.y;
            double dist_sqr = dx * dx + dy * dy;
            if (dist_sqr < dist_min_sqr) {
                dist_min_sqr = dist_sqr;
                count_min = i;
            }
        }
        for (int32_t i = count_min; i < lateral_extended_line.size(); ++i) {
            curvature_line.emplace_back(lateral_extended_line[i]);
        }
    }
}

void SearchNearestLaneLineCount(const CenterLine &lane_line, const double y,
                                double &x, int32_t &count) {
    for (int32_t i = 0; i < lane_line.points.size(); ++i) {
        if (y < lane_line.points[i].position_m.y) {
            x = lane_line.points[i].position_m.x;
            count = i;
            break;
        }
    }
}
}  // namespace planning_lib
}  // namespace jarvis