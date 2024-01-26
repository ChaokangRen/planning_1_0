#include "st_boundary_mapper.h"

#include "discretized_path.h"
#include "sglog/sglog.h"
#include "vec2d.h"

namespace jarvis {
namespace planning_lib {
bool StBoundaryMapper::GetStBoundaryPoints(
    const std::vector<PathPoint> &path_points, const Obstacle &obstacle,
    const ReferenceLine &reference_line, std::vector<STPoint> *upper_points,
    std::vector<STPoint> *lower_points) const {
    if (path_points.empty()) {
        SG_INFO("no points in path data_.discretized_path().");
        return false;
    }
    DistcretizedPath distcretize_path;
    int32_t default_num_point = 50;
    if (path_points.size() > 2 * default_num_point) {
        const int ratio = path_points.size() / default_num_point;
        std::vector<PathPoint> sampled_path_points;
        for (int32_t i = 0; i < path_points.size(); ++i) {
            if (i % ratio == 0) {
                sampled_path_points.push_back(path_points[i]);
            }
        }
        distcretize_path.SetPathPoint(sampled_path_points);
    } else {
        distcretize_path.SetPathPoint(path_points);
    }
    for (int i = 0; i < obstacle.Trajectorys().points.size(); ++i) {
        PathPoint path_point;

        path_point.position_m.x =
            obstacle.Trajectorys().points[i].path_point.position_m.x;
        path_point.position_m.y =
            obstacle.Trajectorys().points[i].path_point.position_m.y;
        path_point.theta_rad =
            obstacle.Trajectorys().points[i].path_point.theta_rad;
        path_point.kappa = obstacle.Trajectorys().points[i].path_point.kappa;
        path_point.s_m = obstacle.Trajectorys().points[i].path_point.s_m;

        const Box2d obs_box = obstacle.GetBoundingBox(path_point);

        double trajectory_time =
            obstacle.Trajectorys().points[i].relative_time_s;
        double buffer = 0.5;
        double step_length = lf_;
        for (double path_s = 0.0; path_s < distcretize_path.Length();
             path_s += step_length) {
            const PathPoint curr_adc_path_point = distcretize_path.Evaluate(
                path_s + distcretize_path.StartPoint().s_m);

            if (CheckOverlap(curr_adc_path_point, obs_box, buffer)) {
                double backward_distance = -step_length;
                double forward_distance = vehicle_length_ + vehicle_width_ +
                                          obs_box.Length() + obs_box.Width();
                double default_min_step = 0.1;

                double fine_tuning_step_length =
                    std::fmin(default_min_step,
                              distcretize_path.Length() / default_num_point);
                bool find_low = false;
                bool find_high = false;
                backward_distance = -10;
                double low_s = std::fmax(0.0, path_s + backward_distance);
                double high_s = std::fmin(distcretize_path.Length(),
                                          path_s + forward_distance);
                while (low_s < high_s) {
                    if (find_high && find_low) {
                        break;
                    }
                    if (find_low == false) {
                        const PathPoint &point_low = distcretize_path.Evaluate(
                            low_s + distcretize_path.StartPoint().s_m);

                        double dx = point_low.position_m.x - obs_box.CenterX();
                        double dy = point_low.position_m.y - obs_box.CenterY();

                        if (!CheckOverlap(point_low, obs_box, buffer)) {
                            low_s += fine_tuning_step_length;
                        } else {
                            find_low = true;
                        }
                    }
                    if (find_high == false) {
                        const PathPoint &point_high = distcretize_path.Evaluate(
                            high_s + distcretize_path.StartPoint().s_m);
                        if (!CheckOverlap(point_high, obs_box, buffer)) {
                            high_s -= fine_tuning_step_length;
                        } else {
                            find_high = true;
                        }
                    }
                }
                if (find_high && find_low) {
                    lower_points->emplace_back(
                        low_s - st_boundary_config_.point_extension,
                        trajectory_time);
                    upper_points->emplace_back(
                        high_s + st_boundary_config_.point_extension,
                        trajectory_time);

                    double vel = 0;
                    if (lower_points->size() > 1) {
                        int count = lower_points->size() - 1;
                        vel = ((*lower_points)[count].s() -
                               (*lower_points)[count - 1].s()) /
                              ((*lower_points)[count].t() -
                               (*lower_points)[count - 1].t());
                    }
                }
                break;
            }
        }
    }
    return (lower_points->size() > 1 && upper_points->size() > 1);
}
bool StBoundaryMapper::CheckOverlap(const PathPoint &path_point,
                                    const Box2d &obs_box,
                                    const double buffer) const {
    double left_delta_l = 0.0;
    double right_delta_l = 0.0;

    if (is_change_lane_) {
        if ((adc_sl_boundary_.start_l + adc_sl_boundary_.end_l) / 2.0 > 0.0) {
            // change to right
            left_delta_l = 1.0;
        } else {
            // change to left
            right_delta_l = 1.0;
        }
    }

    Vec2d vec_to_center(0, 0);
    Vec2d center = Vec2d(path_point.position_m.x + vec_to_center.x(),
                         path_point.position_m.y + vec_to_center.y());
    Box2d adc_box = Box2d(center, path_point.theta_rad,
                          vehicle_length_ + 2 * buffer, vehicle_width_ - 0.5);
    return obs_box.HasOverlap(adc_box);
}

}  // namespace planning_lib
}  // namespace jarvis