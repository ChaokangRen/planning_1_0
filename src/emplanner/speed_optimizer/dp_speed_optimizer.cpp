#include "dp_speed_optimizer.h"

#include "dp_st_graph.h"
#include "reference_line.h"
#include "sglog/sglog.h"
#include "st_boundary.h"
#include "st_boundary_mapper.h"
namespace jarvis {
namespace planning_lib {
bool DpSpeedOptimizer::Init(const PlannerConf &planner_conf,
                            const SpeedOptimizerConf &speed_optimizer_conf,
                            const VehicleParams &vehicle_params) {
    planner_conf_ = planner_conf;
    speed_optimizer_conf_ = speed_optimizer_conf;
    lc_limit_speed_ = speed_optimizer_conf_.speed_limit;
    vehicle_params_ = vehicle_params;
    vehicle_length_ = vehicle_params.vehicle_length;
    vehicle_width_ = vehicle_params.vehicle_width;

    return true;
}

bool DpSpeedOptimizer::OptimalSpeed(const std::vector<PathPoint> &path_points,
                                    const std::vector<Obstacle> &obstacles,
                                    const ReferenceLine &reference_line,
                                    const TrajectoryPoint &init_point,
                                    SpeedData *speed_vec,
                                    std::vector<StBoundary> *st_boundarys,
                                    double *speed_cost,
                                    std::string &debuginfo) {
    SLStaticBoundary adc_sl_boundary{
        path_points[0].position_m.x - vehicle_length_ / 2,
        path_points[0].position_m.x + vehicle_length_ / 2,
        path_points[0].position_m.y + vehicle_width_ / 2,
        path_points[0].position_m.y - vehicle_width_ / 2};
    StBoundaryMapper st_boundary_mapper(adc_sl_boundary);
    DebugInfo debug_info;
    std::string st_bound_str = "{st_bound:";
    for (const Obstacle &obs : obstacles) {
        if (obs.IsStatic() == true) continue;
        std::vector<STPoint> upper_points;
        std::vector<STPoint> lower_points;
        st_boundary_mapper.GetStBoundaryPoints(path_points, obs, reference_line,
                                               &upper_points, &lower_points);

        st_bound_str += "<";
        st_bound_str += "@";
        for (int i = 0; i < lower_points.size(); ++i) {
            st_bound_str += "(" + std::to_string(lower_points[i].s()) + "," +
                            std::to_string(lower_points[i].t()) + "),";
        }
        st_bound_str += "@,";
        st_bound_str += "@";
        for (int i = 0; i < upper_points.size(); ++i) {
            st_bound_str += "(" + std::to_string(upper_points[i].s()) + "," +
                            std::to_string(upper_points[i].t()) + "),";
        }
        st_bound_str += "@";
        st_bound_str += ">,";

        // for (int32_t i = 0; i < lower_points.size(); ++i) {
        //     SG_INFO("i = %d,lo = %lf,t = %lf", i, lower_points[i].s(),
        //             upper_points[i].s());
        // }
        StBoundary st_boundary =
            StBoundary::GenerateStBoundary(lower_points, upper_points);
        st_boundary.SetSpeed(obs.Speed());
        st_boundary.SetIsVirtual(obs.IsVirtual());
        st_boundary.SetId(obs.Id());
        const auto sl_obs = obs.GetObsSlBoundary();
        if (sl_obs.start_s <= 0 && sl_obs.end_s <= 0) {
            st_boundary.SetIsFrontObs(false);
        } else {
            st_boundary.SetIsFrontObs(true);
        }

        st_boundarys->emplace_back(st_boundary);
    }
    st_bound_str += "st_bound_end}";
    debuginfo += st_bound_str;
    // debug_info.SetDebugInfo(st_bound_str);
    DpStGraph dp_st_graph(*st_boundarys, obstacles, init_point, planner_conf_,
                          speed_optimizer_conf_, vehicle_params_);
    dp_st_graph.SetLcLimitSpeed(lc_limit_speed_);
    dp_st_graph.Search(path_points, speed_vec, speed_cost);
    return true;
}
}  // namespace planning_lib
}  // namespace jarvis