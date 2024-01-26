#include "dp_st_graph.h"

#include <algorithm>

#include "line_segment2d.h"
#include "sglog/sglog.h"
#include "st_point.h"
namespace jarvis {
namespace planning_lib {

static bool is_follow_speed = false;
static std::string follow_id;

bool DpStGraph::Init(const PlannerConf &planner_conf,
                     const SpeedOptimizerConf &speed_optimizer_conf,
                     const VehicleParams &vehicle_params) {
    dimension_s_ = speed_optimizer_conf.dimension_s;

    dimension_t_ = static_cast<int>(planner_conf.total_time) + 1;

    unit_s_ = speed_optimizer_conf.unit_s;

    unit_t_ = speed_optimizer_conf.unit_t;

    safe_time_buffer_ = speed_optimizer_conf.safe_time_buffer;

    speed_range_buffer_ = speed_optimizer_conf.speed_range_buffer;

    safe_distance_ = speed_optimizer_conf.safe_distance;

    obstacle_weight_ = speed_optimizer_conf.obstacle_weight;

    speed_weight_ = speed_optimizer_conf.speed_weight;

    accel_weight_ = speed_optimizer_conf.accel_weight;

    jerk_weight_ = speed_optimizer_conf.jerk_weight;

    speed_limit_ = speed_optimizer_conf.speed_limit;

    lc_limit_speed_ = speed_optimizer_conf.speed_limit;

    max_acceleration_ = vehicle_params.max_acceleration;

    max_deceleration_ = vehicle_params.max_deceleartion;

    front_edge_to_center_ = vehicle_params.front_edge_to_center;

    back_edge_to_center_ = vehicle_params.back_edge_to_center;

    max_centric_acc_limit_ = vehicle_params.max_centric_acc_limit;

    minimal_kappa_ = vehicle_params.minimal_kappa;

    return true;
}

DpStGraph::DpStGraph(const std::vector<StBoundary> &st_boundarys,
                     const std::vector<Obstacle> &obstacles,
                     const TrajectoryPoint &init_point,
                     const PlannerConf &planner_conf,
                     const SpeedOptimizerConf &speed_optimizer_conf,
                     const VehicleParams &vehicle_params)
    : st_boundarys_(st_boundarys), init_point_(init_point) {
    Init(planner_conf, speed_optimizer_conf, vehicle_params);
    for (const Obstacle &obs : obstacles) {
        if (obs.IsStatic() == false) {
            obstacles_.emplace_back(&obs);
        }
    }
    obs_boundry_ = std::vector<std::vector<std::pair<double, double>>>(
        st_boundarys_.size(), std::vector<std::pair<double, double>>(
                                  dimension_t_, std::make_pair(-1.0, -1.0)));
}

bool DpStGraph::Search(const std::vector<PathPoint> &path_points,
                       SpeedData *speed_vector, double *total_cost) {
    GetSpeedLimits(path_points);
    if (!InitCostTable()) {
        return false;
    }
    if (!CalculateTotalCost()) {
        return false;
    }
    std::vector<StGraphPoint> &cost_table_col = cost_table_.back();
    double min_cost = std::numeric_limits<double>::infinity();
    StGraphPoint &min_point = cost_table_col.front();
    for (StGraphPoint &ite : cost_table_col) {
        if (min_cost > ite.TotalCost()) {
            min_cost = ite.TotalCost();
            min_point = ite;
        }
    }
    // for (int32_t i = 0; i < cost_table_.size(); ++i) {
    //     for (int32_t j = 0; j < cost_table_[i].size(); ++j) {
    //         SG_INFO("i = %d,j = %d,cost = %lf,obs = %lf", i, j,
    //                 cost_table_[i][j].TotalCost(),
    //                 cost_table_[i][j].ObstacleCost());
    //     }
    // }
    const StGraphPoint *min_point_ptr = &min_point;
    std::vector<SpeedPoint> speed_vec;
    *total_cost = min_point_ptr->ObstacleCost();
    for (int32_t i = 0; i < cost_table_col.size(); ++i) {
        double velocity = 0;
        double s1 = double(min_point_ptr->IndexS());
        double t1 = double(min_point_ptr->IndexT());
        if (min_point_ptr->IndexT() != 0) {
            double s0 = 0;
            double t0 = t1 - unit_t_;
            if (min_point_ptr->PrePoint() != nullptr) {
                s0 = double(min_point_ptr->PrePoint()->IndexS());
                t0 = double(min_point_ptr->PrePoint()->IndexT());
            } else {
            }
            velocity = (s1 - s0) / (t1 - t0);
        } else {
            velocity = init_point_.velocity_mps;
        }
        // *total_cost += min_point_ptr->TotalCost();
        *total_cost += min_point_ptr->ObstacleCost();

        speed_vec.emplace_back(SpeedPoint{s1, t1, velocity, 0, 0});
        min_point_ptr = min_point_ptr->PrePoint();
        if (min_point_ptr == nullptr) {
            break;
        }
    }
    std::reverse(speed_vec.begin(), speed_vec.end());
    speed_vector->SetSpeedVector(speed_vec);
    return true;
}
bool DpStGraph::InitCostTable() {
    cost_table_ = std::vector<std::vector<StGraphPoint>>(
        dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

    float curr_t = 0.0;
    for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
        std::vector<StGraphPoint> &cost_table_i = cost_table_[i];
        float curr_s = 0.0;
        for (uint32_t j = 0; j < cost_table_i.size(); ++j, curr_s += unit_s_) {
            cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
        }
    }
    return true;
}

bool DpStGraph::CalculateTotalCost() {
    uint32_t next_highset_row = 0;
    uint32_t next_lowest_row = 0;
    uint32_t row_max = cost_table_.back().size();

    for (size_t c = 0; c < cost_table_.size(); ++c) {
        int highest_row = 0;
        int lowest_row = cost_table_.back().size() - 1;
        for (uint32_t r = next_lowest_row; r <= next_highset_row; ++r) {
            CalculateCostAt(c, r);
            // SG_INFO("c = %d,r = %d,cost = %lf,obs = %lf", c, r,
            //         cost_table_[c][r].TotalCost(),
            //         cost_table_[c][r].ObstacleCost());
        }
        for (uint32_t r = next_lowest_row; r <= next_highset_row; ++r) {
            const StGraphPoint &cost_cr = cost_table_[c][r];
            if (cost_cr.TotalCost() < std::numeric_limits<float>::infinity()) {
                int h_r = 0;
                int l_r = 0;
                GetRowRange(cost_cr, &h_r, &l_r);
                highest_row = std::max(highest_row, h_r);
                lowest_row = std::min(lowest_row, l_r);
            } else {
                lowest_row = next_lowest_row;
                highest_row = next_highset_row + 1;
            }
        }
        next_highset_row = highest_row;
        next_lowest_row = lowest_row;
        if (next_highset_row >= row_max) {
            next_highset_row = row_max - 1;
        }
    }
    return true;
}
void DpStGraph::CalculateCostAt(const uint32_t c, const uint32_t r) {
    StGraphPoint &cost_cr = cost_table_[c][r];

    cost_cr.SetObstacleCost(GetStObstacleCost(cost_cr));
    const StGraphPoint &cost_init = cost_table_[0][0];
    double cost_sum = 0;
    if (c == 0) {
        cost_cr.SetTotalCost(0.0);
        return;
    }
    // double speed_limit = speed_limit_;
    double speed_limit = speed_limit_vec_[c];
    if (c == 1) {
        float acc =
            (r * unit_s_ / unit_t_ - init_point_.velocity_mps) / unit_t_;
        if (acc > max_acceleration_ || acc < -max_deceleration_) {
            return;
        }

        if (CheckOverlapOnDpStGraph(cost_cr, cost_init)) {
            return;
        }
        cost_cr.SetTotalCost(cost_cr.ObstacleCost() + cost_init.TotalCost() +
                             CalculateEdgeCostForSecondCol(r, speed_limit));
        cost_cr.SetPrePoint(cost_init);
        return;
    }

    uint32_t max_s_diff = static_cast<uint32_t>(
        speed_limit_ * (1 + speed_range_buffer_) * unit_t_ / unit_s_);
    uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);
    const std::vector<StGraphPoint> &pre_col = cost_table_[c - 1];
    bool is_set_pre_point = false;
    if (c == 2) {
        for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
            const float acc =
                (r * unit_s_ - 2 * r_pre * unit_s_) / (unit_t_ * unit_t_);
            if (acc > max_acceleration_ || acc < -max_deceleration_) {
                continue;
            }

            if (CheckOverlapOnDpStGraph(cost_cr, pre_col[r_pre])) {
                continue;
            }

            const float cost =
                cost_cr.ObstacleCost() + pre_col[r_pre].TotalCost() +
                CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);

            if (cost < cost_cr.TotalCost()) {
                cost_cr.SetTotalCost(cost);
                cost_cr.SetPrePoint(pre_col[r_pre]);
                is_set_pre_point = true;
            }
        }
        if (is_set_pre_point == false) {
            float cost = cost_cr.ObstacleCost() + pre_col[r_low].TotalCost() +
                         CalculateEdgeCostForThirdCol(r, r_low, speed_limit);
            cost_cr.SetTotalCost(cost);
            cost_cr.SetPrePoint(pre_col[r_low]);
        }
        return;
    }
    is_set_pre_point = false;
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
        if (std::isinf(pre_col[r_pre].TotalCost()) ||
            pre_col[r_pre].PrePoint() == nullptr) {
            continue;
        }

        const float curr_acc = (cost_cr.IndexS() * unit_s_ +
                                pre_col[r_pre].PrePoint()->IndexS() * unit_s_ -
                                2 * pre_col[r_pre].IndexS() * unit_s_) /
                               (unit_t_ * unit_t_);

        if (curr_acc > max_acceleration_ || curr_acc < -max_deceleration_) {
            continue;
        }

        if (CheckOverlapOnDpStGraph(cost_cr, pre_col[r_pre])) {
            continue;
        }
        uint32_t r_prepre = pre_col[r_pre].PrePoint()->IndexS();
        const StGraphPoint &prepre_graph_point = cost_table_[c - 2][r_prepre];
        if (std::isinf(prepre_graph_point.TotalCost())) {
            continue;
        }
        if (!prepre_graph_point.PrePoint()) {
            continue;
        }
        const STPoint &triple_PrePoint = prepre_graph_point.PrePoint()->Point();
        const STPoint &prePrePoint = prepre_graph_point.Point();
        const STPoint &PrePoint = pre_col[r_pre].Point();
        const STPoint &curr_point = cost_cr.Point();

        float cost = cost_cr.ObstacleCost() + pre_col[r_pre].TotalCost() +
                     CalculateEdgeCost(triple_PrePoint, prePrePoint, PrePoint,
                                       curr_point, speed_limit);

        if (cost < cost_cr.TotalCost()) {
            cost_cr.SetTotalCost(cost);
            cost_cr.SetPrePoint(pre_col[r_pre]);
            is_set_pre_point = true;
        }
    }
    if (is_set_pre_point == false) {
        float cost = cost_cr.ObstacleCost() + pre_col[r_low].TotalCost() +
                     CalculateEdgeCostForThirdCol(r, r_low, speed_limit);
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_low]);
    }
}
double DpStGraph::GetSpeedCost(const STPoint &first, const STPoint &second,
                               const float speed_limit) {
    float cost = 0.0;
    const float speed = (second.s() - first.s()) / unit_t_;
    if (speed < 0) {
        return std::numeric_limits<float>::infinity();
    }
    float det_speed = (speed - speed_limit) / speed_limit;
    if (det_speed > 0) {
        cost += speed_weight_ * std::fabs(speed * speed) * unit_t_;
    } else {
        cost += speed_weight_ * std::pow((speed_limit - speed), 2);
    }
    return cost;
}
double DpStGraph::GetAccelCostByThreePoints(const STPoint &first,
                                            const STPoint &second,
                                            const STPoint &thrid) {
    float accel =
        (first.s() + thrid.s() - 2 * second.s()) / (unit_t_ * unit_t_);
    return accel_weight_ * accel * accel;
}
double DpStGraph::GetAccelCostByTwoPoints(const float pre_speed,
                                          const STPoint &PrePoint,
                                          const STPoint &curr_point) {
    float current_speed = (curr_point.s() - PrePoint.s()) / unit_t_;
    float accel = (current_speed - pre_speed) / unit_t_;
    return accel_weight_ * accel * accel;
}

double DpStGraph::GetJerkCostByFourPoints(const STPoint &first,
                                          const STPoint &second,
                                          const STPoint &third,
                                          const STPoint &fourth) {
    float jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                 (std::pow(unit_t_, 3));
    return jerk_weight_ * jerk * jerk;
}

double DpStGraph::GetStObstacleCost(const StGraphPoint &st_graph_point) {
    double curr_s = st_graph_point.Point().s();
    double curr_t = st_graph_point.Point().t();

    double obs_cost = 0;
    std::vector<int32_t> obs_counts;
    if (!HasObstacleNearCurrPoint(curr_t, &obs_counts)) {
        return obs_cost;
    } else {
        if (HasCollision(st_graph_point.Point())) {
            obs_cost = std::numeric_limits<float>::infinity();

            return obs_cost;
        } else {
            obs_cost = CalculateStObstacleCost(obs_counts, curr_s, curr_t,
                                               st_graph_point.IndexT());
        }
        return obs_cost;
    }
    return obs_cost;
}

double DpStGraph::GetJerkCostByTwoPoints(const float pre_speed,
                                         const float pre_acc,
                                         const STPoint &PrePoint,
                                         const STPoint &curr_point) {
    const float curr_speed = (curr_point.s() - PrePoint.s()) / unit_t_;
    const float curr_accel = (curr_speed - pre_speed) / unit_t_;
    const float jerk = (curr_accel - pre_acc) / unit_t_;
    return jerk_weight_ * jerk * jerk;
}
float DpStGraph::GetJerkCostByThreePoints(const float first_speed,
                                          const STPoint &first,
                                          const STPoint &second,
                                          const STPoint &third) {
    const float pre_speed = (second.s() - first.s()) / unit_t_;
    const float pre_acc = (pre_speed - first_speed) / unit_t_;
    const float curr_speed = (third.s() - second.s()) / unit_t_;
    const float curr_acc = (curr_speed - pre_speed) / unit_t_;
    const float jerk = (curr_acc - pre_acc) / unit_t_;
    return jerk_weight_ * jerk * jerk;
}
double DpStGraph::CalculateStObstacleCost(
    const std::vector<int32_t> &obs_counts, const double curr_s,
    const double curr_t, const int32_t IndexT) {
    double s_upper = 0;
    double s_lower = 0;
    double obs_cost = 0;

    for (const int32_t &counts : obs_counts) {
        if (obs_boundry_[counts][IndexT].first < 0.0) {
            st_boundarys_[counts].GetBoundarySRange(curr_t, &s_upper, &s_lower);
            obs_boundry_[counts][IndexT] = std::make_pair(s_upper, s_lower);
        } else {
            s_upper = obs_boundry_[counts][IndexT].first;
            s_lower = obs_boundry_[counts][IndexT].second;
        }
        // mean obs in front of adc
        if (curr_s < s_lower) {
            // float len = obstacles_[counts]->Speed() * safe_time_buffer_;
            float vel = obstacles_[counts]->Speed() > init_point_.velocity_mps
                            ? obstacles_[counts]->Speed()
                            : init_point_.velocity_mps;
            float len = init_point_.velocity_mps * safe_time_buffer_;
            // SG_INFO("len = %lf,currs = %lf,vel = %lf,t = %lf", len, s_lower,
            //         obstacles_[counts]->Speed(), safe_time_buffer_);

            if (curr_s + len < s_lower) {
                continue;
            } else {
                obs_cost +=
                    obstacle_weight_ * std::pow((len - s_lower + curr_s), 2);
            }
        } else if (curr_s > s_upper) {
            if (curr_s > s_upper + safe_distance_) {
                continue;
            } else {
                obs_cost += obstacle_weight_ *
                            std::pow(s_upper + safe_distance_ - curr_s, 2);
            }
        }
    }

    return obs_cost;
}
bool DpStGraph::HasObstacleNearCurrPoint(const double curr_t,
                                         std::vector<int32_t> *obs_counts) {
    bool has_obs_near = false;
    for (int32_t i = 0; i < st_boundarys_.size(); ++i) {
        if (curr_t > st_boundarys_[i].MinT() &&
            curr_t <= st_boundarys_[i].MaxT()) {
            obs_counts->push_back(i);
            has_obs_near = true;
        }
    }
    return has_obs_near;
}
bool DpStGraph::HasCollision(const STPoint &st_point) {
    double curr_s = st_point.s();
    double curr_t = st_point.t();
    double step_length = 0.5;
    double safety_dist = 0.0;
    for (const StBoundary &st_obstacle : st_boundarys_) {
        front_edge_to_center_ = 0.5;
        back_edge_to_center_ = 0.5;
        for (float len = 0; len < front_edge_to_center_ + safety_dist;
             len += step_length) {
            if (st_obstacle.IsPointInBoundary(STPoint(curr_s + len, curr_t))) {
                return true;
            }
        }
        for (float len = 0; len < back_edge_to_center_ + safety_dist;
             len += step_length) {
            if (st_obstacle.IsPointInBoundary(STPoint(curr_s - len, curr_t))) {
                return true;
            }
        }
    }
    return false;
}
void DpStGraph::GetRowRange(const StGraphPoint &point, int *next_highest_row,
                            int *next_lowest_row) {
    float v0 = 0.0;
    if (!point.PrePoint()) {
        v0 = init_point_.velocity_mps;
    } else {
        v0 = (point.IndexS() - point.PrePoint()->IndexS()) * unit_s_ / unit_t_;
    }

    const int max_s_size = cost_table_.back().size() - 1;
    const float speed_coeff = unit_t_ * unit_t_;
    const float delta_s_upper_bound =
        v0 * unit_t_ + max_acceleration_ * speed_coeff;
    *next_highest_row =
        point.IndexS() + static_cast<int>(delta_s_upper_bound / unit_s_);
    if (*next_highest_row >= max_s_size) {
        *next_highest_row = max_s_size;
    }

    const float delta_s_lower_bound =
        std::fmax(0.0, v0 * unit_t_ - max_deceleration_ * speed_coeff);
    *next_lowest_row =
        point.IndexS() + static_cast<int>(delta_s_lower_bound / unit_s_);
    if (*next_lowest_row > max_s_size) {
        *next_lowest_row = max_s_size;
    } else if (*next_lowest_row < 0) {
        *next_lowest_row = 0;
    }
}

bool DpStGraph::CheckOverlapOnDpStGraph(const StGraphPoint &p1,
                                        const StGraphPoint &p2) {
    const LineSegment2d seg(p1.Point(), p2.Point());
    for (const StBoundary &boundry : st_boundarys_) {
        if (boundry.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
            continue;
        }
        if (boundry.HasOverlap(seg)) {
            return true;
        }
    }
    return false;
}

float DpStGraph::CalculateEdgeCost(const STPoint &first, const STPoint &second,
                                   const STPoint &third, const STPoint &forth,
                                   const float speed_limit) {
    return GetSpeedCost(third, forth, speed_limit) +
           GetAccelCostByThreePoints(second, third, forth) +
           GetJerkCostByFourPoints(first, second, third, forth);
}
float DpStGraph::CalculateEdgeCostForSecondCol(const uint32_t row,
                                               const float speed_limit) {
    float init_speed = init_point_.velocity_mps;
    float init_acc = init_point_.acceleration;
    const STPoint &PrePoint = cost_table_[0][0].Point();
    const STPoint &curr_point = cost_table_[1][row].Point();
    // SG_WARN(
    //     "row = %d,speed cost = %lf,acc cost = %lf,jerk cost = %lf,speed limit
    //     "
    //     "= %lf",
    //     row, GetSpeedCost(PrePoint, curr_point, speed_limit),
    //     GetAccelCostByTwoPoints(init_speed, PrePoint, curr_point),
    //     GetJerkCostByTwoPoints(init_speed, init_acc, PrePoint, curr_point),
    //     speed_limit);

    return GetSpeedCost(PrePoint, curr_point, speed_limit) +
           GetAccelCostByTwoPoints(init_speed, PrePoint, curr_point) +
           GetJerkCostByTwoPoints(init_speed, init_acc, PrePoint, curr_point);
}
float DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t row,
                                              const uint32_t pre_row,
                                              const float speed_limit) {
    float init_speed = init_point_.velocity_mps;
    const STPoint &first = cost_table_[0][0].Point();
    const STPoint &second = cost_table_[1][pre_row].Point();
    const STPoint &thrid = cost_table_[2][row].Point();

    return GetSpeedCost(second, thrid, speed_limit) +
           GetAccelCostByThreePoints(first, second, thrid) +
           GetJerkCostByThreePoints(init_speed, first, second, thrid);
}

bool DpStGraph::GetSpeedLimits(const std::vector<PathPoint> &path_points) {
    double obs_follow_speed = 50;
    IsNeedFollowSpeed(obs_follow_speed);
    double speed_tmp = 0.0;
    for (int32_t i = 0; i < path_points.size(); ++i) {
        double speed = std::sqrt(
            max_centric_acc_limit_ /
            std::fmax(std::fabs(path_points[i].kappa), minimal_kappa_));
        double min_speed =
            std::fmin(speed_limit_, std::fmin(speed, path_points[i].dkappa));
        // double tmp_speed = 9.7222;
        min_speed = std::fmin(obs_follow_speed, min_speed);
        // SG_INFO("kappa speed = %lf,obs_follow_speed = %lf,min_speed = %lf",
        //         path_points[i].dkappa, obs_follow_speed, min_speed);
        // if (min_speed - init_point_.velocity_mps > 2.0) {
        //     min_speed = init_point_.velocity_mps + 2.0;
        //     // SG_INFO("min speed = %lf",min_speed);
        // }
        min_speed = std::fmin(min_speed, lc_limit_speed_);
        speed_limit_vec_.emplace_back(min_speed);
        // speed_limit_vec_.emplace_back(tmp_speed);
    }
    return true;
}

bool DpStGraph::IsNeedFollowSpeed(double &follow_speed) {
    follow_speed = 50;
    bool has_follow_id = false;

    if (is_follow_speed == true) {
        // SG_INFO("has follow");
    } else {
        // SG_INFO("no follow");
    }
    if (is_follow_speed == true) {
        // SG_INFO("st size = %d", st_boundarys_.size());
        for (int32_t i = 0; i < st_boundarys_.size(); ++i) {
            // SG_INFO("st bouddary size = %d",
            //         st_boundarys_[i].lower_points().size());

            // SG_INFO("follow id = %s,st id = %s", follow_id.c_str(),
            //         st_boundarys_[i].id().c_str());

            if (follow_id == st_boundarys_[i].id()) {
                if (st_boundarys_[i].lower_points().empty() == false) {
                    if (st_boundarys_[i].lower_points().front().s() < 50) {
                        follow_speed = st_boundarys_[i].GetSpeed();
                        has_follow_id = true;
                        break;
                    }
                }
            }
        }
        if (follow_speed <= 6.5) {
            follow_speed = 6.5;
        }
        if (has_follow_id == true) {
            is_follow_speed = true;
        } else {
            is_follow_speed = false;
        }
    }
    if (is_follow_speed == false) {
        for (int32_t i = 0; i < st_boundarys_.size(); ++i) {
            if (st_boundarys_[i].lower_points().empty() == false) {
                // SG_INFO("frontx s = %lf",
                //         st_boundarys_[i].lower_points().front().s());
                if (st_boundarys_[i].lower_points().front().t() < 1.0 &&
                    st_boundarys_[i].lower_points().back().t() > 3 &&
                    st_boundarys_[i].lower_points().front().s() < 25) {
                    // SG_INFO("has follow speed");
                    is_follow_speed = true;
                    follow_id = st_boundarys_[i].id();
                    follow_speed = st_boundarys_[i].GetSpeed();
                    if (follow_speed <= 6.5) {
                        follow_speed = 6.5;
                    }
                }
            }
        }
    }

    // SG_WARN("follow speed = %lf", follow_speed);
    return true;
}

// bool DpStGraph::IsNeedFollowSpeed(double &follow_speed) {
//     follow_speed = 50.0;
//     double min_boundary_dist = 15;
//     for (int32_t i = 0; i < st_boundarys_.size(); ++i) {
//         if (st_boundarys_[i].lower_points().empty() == false) {
//             if (st_boundarys_[i].lower_points().front().t() < 1.0 &&
//                 st_boundarys_[i].lower_points().back().t() > 3 &&
//                 st_boundarys_[i].lower_points().front().s() <
//                     min_boundary_dist) {
//                 if (st_boundarys_[i].GetIsVitual() == true) {
//                     if (st_boundarys_[i].lower_points().front().s() < 6) {
//                         follow_speed = st_boundarys_[i].GetSpeed();
//                     } else {
//                         continue;
//                     }
//                 } else {
//                     if (st_boundarys_[i].IsFrontObs() == false) {
//                         continue;
//                     }
//                     follow_speed = st_boundarys_[i].GetSpeed();
//                     if (follow_speed < 3 &&
//                         st_boundarys_[i].lower_points().front().s() > 6) {
//                         follow_speed = 4.0;
//                     }
//                 }
//             }
//         }
//     }
//     SG_INFO("follow speed = %lf", follow_speed);
//     return true;
// }

// bool DpStGraph::IsNeedFollowSpeed(double &follow_speed) {
//     follow_speed = 50.0;
//     double min_boundary_dist = 4.5;
//     for (int32_t i = 0; i < st_boundarys_.size(); ++i) {
//         if (st_boundarys_[i].lower_points().empty() == false) {
//             if (st_boundarys_[i].lower_points().front().t() < 1.0 &&
//                 st_boundarys_[i].lower_points().back().t() > 3 &&
//                 st_boundarys_[i].lower_points().front().s() <
//                     min_boundary_dist) {
//                 follow_speed = st_boundarys_[i].GetSpeed();
//             }
//         }
//     }
//     SG_INFO("follow speed = %lf", follow_speed);
//     return true;
// }

}  // namespace planning_lib
}  // namespace jarvis