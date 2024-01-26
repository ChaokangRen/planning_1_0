#include "path_bounds_decider.h"
namespace jarvis {
namespace planning_lib {

void PathBoundsDecider::Process(
    const SpeedData &heuristic_speed_data,
    const std::vector<Obstacle> &obstacles, const DpPathData &dp_path,
    const ReferenceLine *refline,
    std::vector<std::pair<LaneLine, LaneLine>> &sl_navigable_areas) {
    heuristic_speed_data_ = &heuristic_speed_data;
    // SG_WARN("lc_dir=%d", lc_direction_);
    LaneLine left_navigable_area_;
    LaneLine right_navigable_area_;

    // if (lc_direction_ == LcDirection::LaneKeep) {
    //     SetStraightBound(obstacles, dp_path, refline, left_navigable_area_,
    //                      right_navigable_area_);
    // } else {
    //     SetLcBound(obstacles, dp_path, refline, left_navigable_area_,
    //                right_navigable_area_);
    // }

    SetBound(obstacles, dp_path, refline, left_navigable_area_,
             right_navigable_area_);

    sl_navigable_areas.clear();
    sl_navigable_areas.emplace_back(
        std::make_pair(left_navigable_area_, right_navigable_area_));
    // SG_WARN("l_bound=%f,right_bound=%f",
    //         left_navigable_area_.GetLaneLineBoundaryByIndex(0),
    //         right_navigable_area_.GetLaneLineBoundaryByIndex(0));
}

void PathBoundsDecider::SetBound(const std::vector<Obstacle> &obstacles,
                                 const DpPathData &dp_path,
                                 const ReferenceLine *refline,
                                 LaneLine &left_navigable_area_,
                                 LaneLine &right_navigable_area_) {
    // SLPoint sl_left;
    // SLPoint sl_right;
    // left_navigable_area_.emplace_back();
    for (int i = 0; i < dp_path.size(); i++) {
        double s = dp_path[i].pos.s;
        double l = dp_path[i].pos.l;
        // SG_WARN("bound_s=%f,bound_l=%f", s, l);
        //  cal from ref
        double left_bound = 1.75;
        double right_bound = -1.75;
        if (i != 0) {
            CalculateQpBoundaryConstraint(obstacles, refline, dp_path[i].pos,
                                          &left_bound, &right_bound);
        }
        SLPoint sl_left;
        SLPoint sl_right;
        sl_left.l = left_bound;
        sl_left.s = s;
        sl_right.l = right_bound;
        sl_right.s = s;
        left_navigable_area_.emplace_back(sl_left);
        right_navigable_area_.emplace_back(sl_right);
    }
}

void PathBoundsDecider::SetStraightBound(const std::vector<Obstacle> &obstacles,
                                         const DpPathData &dp_path,
                                         const ReferenceLine *refline,
                                         LaneLine &left_navigable_area_,
                                         LaneLine &right_navigable_area_) {
    // SLPoint sl_left;
    // SLPoint sl_right;
    // left_navigable_area_.emplace_back();
    for (int i = 0; i < dp_path.size(); i++) {
        double s = dp_path[i].pos.s;
        double l = dp_path[i].pos.l;
        // SG_WARN("bound_s=%f,bound_l=%f", s, l);
        //  cal from ref
        double left_bound = 1.75;
        double right_bound = -1.75;
        if (i != 0) {
            CalculateQpBoundaryConstraint(obstacles, refline, dp_path[i].pos,
                                          &left_bound, &right_bound);
        }
        SLPoint sl_left;
        SLPoint sl_right;
        sl_left.l = left_bound;
        sl_left.s = s;
        sl_right.l = right_bound;
        sl_right.s = s;
        left_navigable_area_.emplace_back(sl_left);
        right_navigable_area_.emplace_back(sl_right);
    }
}

void PathBoundsDecider::SetLcBound(const std::vector<Obstacle> &obstacles,
                                   const DpPathData &dp_path,
                                   const ReferenceLine *refline,
                                   LaneLine &left_navigable_area_,
                                   LaneLine &right_navigable_area_) {
    int count_i = 1000;
    bool is_first_time = false;
    double l0 = dp_path[0].pos.l;

    for (int i = 0; i < dp_path.size() - 1; ++i) {
        double s = dp_path[i].pos.s;
        double l = dp_path[i].pos.l;
        double l_next = dp_path[i + 1].pos.l;
        // cal from ref
        double left_bound;
        double right_bound;
        if (lc_direction_ == LcDirection::TurnLeft) {
            left_bound = 5.5;
            right_bound = -1.75;
            // if ((l - cross_point_l_) * (l_next - cross_point_l_) < 0 ||
            //     i > count_i) {
            //     if (!is_first_time) {
            //         if (i + 5 < dp_path.size()) {
            //             count_i = i + 5;
            //         } else {
            //             count_i = dp_path.size() - 1;
            //         }
            //         is_first_time = true;
            //     } else {
            //         right_bound = 1.75;
            //     }
            //     right_bound = 1.75;
            // } else if (l0 > cross_point_l_) {
            //     right_bound = 1.75;
            // }
        } else if (lc_direction_ == LcDirection::TurnRight) {
            left_bound = 1.75;
            right_bound = -5.5;
            // if ((l - cross_point_l_) * (l_next - cross_point_l_) < 0 ||
            //     i > count_i) {
            //     if (!is_first_time) {
            //         if (i + 5 < dp_path.size()) {
            //             count_i = i + 5;
            //         } else {
            //             count_i = dp_path.size() - 1;
            //         }
            //         is_first_time = true;
            //     } else {
            //         left_bound = -1.75;
            //     }
            //     // SG_WARN("1=%f,2=%d",
            //     //         (l - cross_point_l_) * (l_next - cross_point_l_),
            //     //         is_first_time);

            // } else if (l0 < cross_point_l_) {
            //     left_bound = -1.75;
            // }
        }
        // SG_WARN("cross_l=%f,count_i=%d", cross_point_l_, count_i);
        // SG_WARN("dp_path_l=%f,dp_path_s=%f,l_bound=%f", dp_path[i].pos.l,
        //         dp_path[i].pos.s, left_bound);
        if (i != 0) {
            CalculateQpBoundaryConstraint(obstacles, refline, dp_path[i].pos,
                                          &left_bound, &right_bound);
        }
        SLPoint sl_left;
        SLPoint sl_right;
        sl_left.l = left_bound;
        sl_left.s = s;
        sl_right.l = right_bound;
        sl_right.s = s;
        // SG_INFO("sl_left=%f,sl_right-%f", sl_left.l, sl_right.l);
        left_navigable_area_.emplace_back(sl_left);
        right_navigable_area_.emplace_back(sl_right);
    }
    left_navigable_area_.emplace_back(left_navigable_area_.GetLine().back());
    right_navigable_area_.emplace_back(right_navigable_area_.GetLine().back());
}

void PathBoundsDecider::CalculateQpBoundaryConstraint(
    const std::vector<Obstacle> &obstacles, const ReferenceLine *refline,
    const SLPoint &ref_point, double *left_boundary, double *right_boundary) {
    double left_width = *left_boundary;
    double right_width = *right_boundary;

    double adc_width_half = vehicle_width_ / 2;
    double left_road_boundary = left_width - adc_width_half - boundary_buff_;
    double right_road_boundary = right_width + adc_width_half + boundary_buff_;

    // 2 get static obstacle boundary
    // SG_INFO("ref point s = %lf,l = %lf,left_width = %lf,right width = %lf",
    //         ref_point.s, ref_point.l, left_road_boundary,
    //         right_road_boundary);

    for (const Obstacle &obstacle : obstacles) {
        if (obstacle.IsStatic() == true) {
            CalculateStaticObsConstraint(ref_point, obstacle.GetObsSlBoundary(),
                                         &left_road_boundary,
                                         &right_road_boundary);
        }
    }

    // SG_ERROR("left_width = %lf,right width = %lf", left_road_boundary,
    //          right_road_boundary);

    // 3 get dynamic obstacle boundary
    CalculateDynamicObsConstraint(obstacles, refline, ref_point,
                                  &left_road_boundary, &right_road_boundary,
                                  left_width, right_width);

    double ref_upper = ref_point.l + adc_width_half + boundary_buff_;
    double ref_lower = ref_point.l - adc_width_half - boundary_buff_;
    if (ref_upper > left_road_boundary) {
        left_road_boundary = ref_upper;
    }
    if (ref_lower < right_road_boundary) {
        right_road_boundary = ref_lower;
    }

    *left_boundary = left_road_boundary;
    *right_boundary = right_road_boundary;
    // SG_ERROR("*left_width = %lf,*right width = %lf", *left_boundary,
    //          *right_boundary);
}

void PathBoundsDecider::CalculateStaticObsConstraint(
    const SLPoint &ref_point, const SLStaticBoundary &obstacle,
    double *left_boundary, double *right_boundary) {
    // 1 Determine whether the obstacle is near the reference point
    if (ref_point.s - longitudinal_safe_buff_ > obstacle.end_s ||
        ref_point.s + longitudinal_safe_buff_ / 2 < obstacle.start_s) {
        return;
    }
    // 2 Find the obstacle boundary
    if (obstacle.start_l > ref_point.l) {
        *left_boundary = std::min(*left_boundary, obstacle.start_l);
        // *left_boundary =
        //     *left_boundary - vehicle_width_ / 2 - lateral_safe_buff_;
        return;
    }

    if (obstacle.end_l < ref_point.l) {
        *right_boundary = std::max(*right_boundary, obstacle.end_l);
        // *right_boundary =
        //     *right_boundary + vehicle_width_ / 2 + lateral_safe_buff_;
        return;
    }

    // 3 in case point in obstacle
    if (std::abs(obstacle.start_l - ref_point.l) <
        std::abs(obstacle.end_l - ref_point.l)) {
        *left_boundary = std::min(*left_boundary, obstacle.start_l);
        // *left_boundary =
        //     *left_boundary - vehicle_width_ / 2 - lateral_safe_buff_;
    } else {
        *right_boundary = std::max(*right_boundary, obstacle.end_l);
        // *right_boundary =
        //     *right_boundary + vehicle_width_ / 2 + lateral_safe_buff_;
    }
}

void PathBoundsDecider::CalculateDynamicObsConstraint(
    const std::vector<Obstacle> &obstacles, const ReferenceLine *refline,
    const SLPoint &ref_point, double *left_boundary, double *right_boundary,
    double left_lane_boundary, double right_lane_boundary) {
    // SG_ERROR("cal qp bound!!!!!!!!!!!!!!!!!!!!!!!!");
    for (const Obstacle &dynamic_obs : obstacles) {
        if (dynamic_obs.IsStatic()) {
            // SG_INFO("static_id=%s", dynamic_obs.Id().c_str());
            continue;
        }
        // 0 get current time polygon of dynamic obstacle
        double heuristic_time = 0;

        if (!heuristic_speed_data_->EvaluateTimeByS(ref_point.s,
                                                    &heuristic_time)) {
            continue;
        }
        PathPoint obs_point_by_time;
        if (!dynamic_obs.GetPointAtTime(heuristic_time, &obs_point_by_time)) {
            continue;
        }
        double obs_s = 0;
        double obs_l = 0;
        refline->XyToSl(obs_point_by_time.position_m.x,
                        obs_point_by_time.position_m.y, &obs_s, &obs_l);

        // Box2d obstalce_box = Box2d(Vec2d(obs_s, obs_l),
        // obs_point_by_time.theta,
        //                            dynamic_obs.Length(),
        //                            dynamic_obs.Width());
        Box2d obstalce_box =
            Box2d(Vec2d(obs_s, obs_l), 0, dynamic_obs.Length() + 6,
                  dynamic_obs.Width());
        Polygon2d poly_obs = Polygon2d(obstalce_box);

        // 1 Determine whether the obstacle is near the reference point
        double adc_front_s = ref_point.s + vehicle_length_ / 2;
        double adc_rear_s = ref_point.s - vehicle_length_ / 2;
        if (adc_front_s < poly_obs.MinX() || adc_rear_s > poly_obs.MaxX()) {
            continue;
        }
        // LineSegment2d ref_vertical_line(Vec2d(ref_point.s, *left_boundary),
        //                                 Vec2d(ref_point.s, *right_boundary));
        LineSegment2d ref_vertical_line(
            Vec2d(ref_point.s + 2, left_lane_boundary),
            Vec2d(ref_point.s + 2, right_lane_boundary));
        Vec2d intersect_point{0, 0};
        // // 2 The obstacle is to the right of the reference point
        // SG_WARN(
        //     "ref_point_l=%f,ref_point_s=%f,obs_s=%f,poly_max=%f,poly_min=%f",
        //     ref_point.l, ref_point.s, obs_s, poly_obs.MaxY(),
        //     poly_obs.MinY());
        if (ref_point.l > poly_obs.MaxY()) {
            for (const LineSegment2d &poly_line : poly_obs.line_segments()) {
                if (poly_line.HasIntersectWithPoint(ref_vertical_line,
                                                    &intersect_point)) {
                    *right_boundary =
                        std::max(*right_boundary, intersect_point.y() +
                                                      lateral_safe_buff_ +
                                                      vehicle_width_ / 2);
                    // SG_INFO(
                    //     "has intersect_right: point_s "
                    //     "=%f,point_l=%f,inter_point=%f,min_rightt=%f,right_"
                    //     "boundary=%f",
                    //     obs_s, obs_l, intersect_point.y(),
                    //     intersect_point.y() - lateral_safe_buff_ -
                    //         vehicle_width_ / 2,
                    //     *right_boundary);
                }
            }
        }

        // // 3 The obstacle is to the left of the reference point
        if (ref_point.l < poly_obs.MinY()) {
            for (const LineSegment2d &poly_line : poly_obs.line_segments()) {
                // SG_INFO("point_s =%f,point_l=%f", obs_s, obs_l);
                if (poly_line.HasIntersectWithPoint(ref_vertical_line,
                                                    &intersect_point)) {
                    *left_boundary =
                        std::min(*left_boundary, intersect_point.y() -
                                                     lateral_safe_buff_ -
                                                     vehicle_width_ / 2);
                    // SG_INFO(
                    //     "has intersect_left: point_s "
                    //     "=%f,point_l=%f,inter_point=%f,min_left=%f,left_"
                    //     "boundary=%f",
                    //     obs_s, obs_l, intersect_point.y(),
                    //     intersect_point.y() - lateral_safe_buff_ -
                    //         vehicle_width_ / 2,
                    //     *left_boundary);
                    // SG_INFO("bounds_obs_id=%s", dynamic_obs.Id().c_str());
                }
            }
        }
        // SG_INFO(
        //     "ref_l=%f,left_width = %lf,right width = "
        //     "%lf,id=%s,obs_s=%f,obs_l=%f",
        //     ref_point.l, *left_boundary, *right_boundary,
        //     dynamic_obs.Id().c_str(), obs_s, intersect_point.y());
    }
}

}  // namespace planning_lib
}  // namespace jarvis