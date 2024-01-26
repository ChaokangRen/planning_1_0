#include "json_parser.h"

#include <fstream>

#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {
bool JsonParser::ParserPlanningConf(const std::string &path) {
    Json::Reader reader;
    Json::Value root;

    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        SG_ERROR("vehicle_parameter file open failed!");
    }
    if (reader.parse(in, root)) {
        // vehicle_params
        planning_conf_.vehicle_params.front_edge_to_center =
            root["vehicle_params"]["front_edge_to_center"].asDouble();
        planning_conf_.vehicle_params.back_edge_to_center =
            root["vehicle_params"]["back_edge_to_center"].asDouble();
        planning_conf_.vehicle_params.left_edge_to_center =
            root["vehicle_params"]["left_edge_to_center"].asDouble();
        planning_conf_.vehicle_params.right_dege_to_center =
            root["vehicle_params"]["right_dege_to_center"].asDouble();
        planning_conf_.vehicle_params.rear_view_mirror_width =
            root["vehicle_params"]["rear_view_mirror_width"].asDouble();
        planning_conf_.vehicle_params.vehicle_width =
            root["vehicle_params"]["vehicle_width"].asDouble();
        planning_conf_.vehicle_params.vehicle_length =
            root["vehicle_params"]["vehicle_length"].asDouble();
        planning_conf_.vehicle_params.min_turn_radius =
            root["vehicle_params"]["min_turn_radius"].asDouble();
        planning_conf_.vehicle_params.max_acceleration =
            root["vehicle_params"]["max_acceleration"].asDouble();
        planning_conf_.vehicle_params.max_deceleartion =
            root["vehicle_params"]["max_deceleartion"].asDouble();
        planning_conf_.vehicle_params.max_steer_angle =
            root["vehicle_params"]["max_steer_angle"].asDouble();
        planning_conf_.vehicle_params.max_steer_angle_rate =
            root["vehicle_params"]["max_steer_angle_rate"].asDouble();
        planning_conf_.vehicle_params.min_steer_angle_rate =
            root["vehicle_params"]["min_steer_angle_rate"].asDouble();
        planning_conf_.vehicle_params.steer_ratio =
            root["vehicle_params"]["steer_ratio"].asDouble();
        planning_conf_.vehicle_params.wheel_base =
            root["vehicle_params"]["wheel_base"].asDouble();
        planning_conf_.vehicle_params.mass =
            root["vehicle_params"]["mass"].asDouble();
        planning_conf_.vehicle_params.max_centric_acc_limit =
            root["vehicle_params"]["max_centric_acc_limit"].asDouble();
        planning_conf_.vehicle_params.minimal_kappa =
            root["vehicle_params"]["minimal_kappa"].asDouble();

        // reference_line_smooth_conf
        planning_conf_.reference_line_smooth_conf.refline_total_length =
            root["reference_line_smooth_conf"]["refline_total_length"]
                .asDouble();
        planning_conf_.reference_line_smooth_conf.refline_total_num =
            root["reference_line_smooth_conf"]["refline_total_num"].asInt();
        planning_conf_.reference_line_smooth_conf.seg_num =
            root["reference_line_smooth_conf"]["seg_num"].asInt();
        planning_conf_.reference_line_smooth_conf.interval =
            root["reference_line_smooth_conf"]["interval"].asDouble();
        planning_conf_.reference_line_smooth_conf.ref_line_cost_weight =
            root["reference_line_smooth_conf"]["ref_line_cost_weight"]
                .asDouble();
        planning_conf_.reference_line_smooth_conf.qp_param_num =
            root["reference_line_smooth_conf"]["qp_param_num"].asInt();
        planning_conf_.reference_line_smooth_conf.qp_inequ_num =
            root["reference_line_smooth_conf"]["qp_inequ_num"].asInt();
        planning_conf_.reference_line_smooth_conf.qp_weight_1 =
            root["reference_line_smooth_conf"]["qp_weight_1"].asDouble();
        planning_conf_.reference_line_smooth_conf.qp_weight_2 =
            root["reference_line_smooth_conf"]["qp_weight_2"].asDouble();
        planning_conf_.reference_line_smooth_conf.qp_weight_3 =
            root["reference_line_smooth_conf"]["qp_weight_3"].asDouble();
        planning_conf_.reference_line_smooth_conf.qp_weight_4 =
            root["reference_line_smooth_conf"]["qp_weight_4"].asDouble();

        // planner_conf
        planning_conf_.planner_conf.total_length =
            root["planner_conf"]["total_length"].asDouble();
        planning_conf_.planner_conf.total_time =
            root["planner_conf"]["total_time"].asDouble();
        planning_conf_.planner_conf.trajectory_time_interval =
            root["planner_conf"]["trajectory_time_interval"].asDouble();

        // path_optimizer_conf
        // smooth
        planning_conf_.path_optimizer_conf.lateral_sample_num =
            root["path_optimizer_conf"]["lateral_sample_num"].asInt();
        planning_conf_.path_optimizer_conf.longitudinal_sample_num =
            root["path_optimizer_conf"]["longitudinal_sample_num"].asInt();
        planning_conf_.path_optimizer_conf.boundary_buffer =
            root["path_optimizer_conf"]["boundary_buffer"].asDouble();
        planning_conf_.path_optimizer_conf.offset_to_other_referenceline =
            root["path_optimizer_conf"]["offset_to_other_referenceline"]
                .asDouble();
        planning_conf_.path_optimizer_conf.sige_pass_distance =
            root["path_optimizer_conf"]["sige_pass_distance"].asDouble();
        // dp
        planning_conf_.path_optimizer_conf.eval_time_interval =
            root["path_optimizer_conf"]["eval_time_interval"].asDouble();
        planning_conf_.path_optimizer_conf.dp_path_cost_sample_dist =
            root["path_optimizer_conf"]["dp_path_cost_sample_dist"].asFloat();
        planning_conf_.path_optimizer_conf.dp_path_cost =
            root["path_optimizer_conf"]["dp_path_cost"].asFloat();
        planning_conf_.path_optimizer_conf.dp_path_dl_cost =
            root["path_optimizer_conf"]["dp_path_dl_cost"].asFloat();
        planning_conf_.path_optimizer_conf.dp_path_ddl_cost =
            root["path_optimizer_conf"]["dp_path_ddl_cost"].asFloat();
        planning_conf_.path_optimizer_conf.dp_static_obs_cost =
            root["path_optimizer_conf"]["dp_static_obs_cost"].asFloat();
        planning_conf_.path_optimizer_conf.dp_dynamic_obs_cost_small =
            root["path_optimizer_conf"]["dp_dynamic_obs_cost_small"].asFloat();
        planning_conf_.path_optimizer_conf.dp_dynamic_obs_cost_big =
            root["path_optimizer_conf"]["dp_dynamic_obs_cost_big"].asFloat();
        planning_conf_.path_optimizer_conf.has_collision_cost =
            root["path_optimizer_conf"]["has_collision_cost"].asDouble();
        planning_conf_.path_optimizer_conf.safety_distance =
            root["path_optimizer_conf"]["safety_distance"].asDouble();
        planning_conf_.path_optimizer_conf.obstacle_collision_distance =
            root["path_optimizer_conf"]["obstacle_collision_distance"]
                .asDouble();
        planning_conf_.path_optimizer_conf.risk_obstacle_collision_distance =
            root["path_optimizer_conf"]["risk_obstacle_collision_distance"]
                .asDouble();
        planning_conf_.path_optimizer_conf.obstacle_ignore_distance =
            root["path_optimizer_conf"]["obstacle_ignore_distance"].asDouble();
        planning_conf_.path_optimizer_conf.static_obs_safe_ratio =
            root["path_optimizer_conf"]["static_obs_safe_ratio"].asDouble();
        // qp
        planning_conf_.path_optimizer_conf.lane_num =
            root["path_optimizer_conf"]["lane_num"].asInt();
        planning_conf_.path_optimizer_conf.qp_param_num =
            root["path_optimizer_conf"]["qp_param_num"].asInt();
        planning_conf_.path_optimizer_conf.qp_inequ_num =
            root["path_optimizer_conf"]["qp_inequ_num"].asInt();
        planning_conf_.path_optimizer_conf.t_interval =
            root["path_optimizer_conf"]["t_interval"].asDouble();
        planning_conf_.path_optimizer_conf.longitudinal_safe_buffer =
            root["path_optimizer_conf"]["longitudinal_safe_buffer"].asDouble();
        planning_conf_.path_optimizer_conf.lateral_safe_buffer =
            root["path_optimizer_conf"]["lateral_safe_buffer"].asDouble();
        planning_conf_.path_optimizer_conf.qp_weight_1 =
            root["path_optimizer_conf"]["qp_weight_1"].asDouble();
        planning_conf_.path_optimizer_conf.qp_weight_2 =
            root["path_optimizer_conf"]["qp_weight_2"].asDouble();
        planning_conf_.path_optimizer_conf.qp_weight_3 =
            root["path_optimizer_conf"]["qp_weight_3"].asDouble();
        planning_conf_.path_optimizer_conf.qp_weight_4 =
            root["path_optimizer_conf"]["qp_weight_4"].asDouble();

        // speed_optimizer_conf
        // dp
        planning_conf_.speed_optimizer_conf.dimension_s =
            root["speed_optimizer_conf"]["dimension_s"].asInt();
        planning_conf_.speed_optimizer_conf.unit_s =
            root["speed_optimizer_conf"]["unit_s"].asFloat();
        planning_conf_.speed_optimizer_conf.unit_t =
            root["speed_optimizer_conf"]["unit_t"].asFloat();
        planning_conf_.speed_optimizer_conf.safe_time_buffer =
            root["speed_optimizer_conf"]["safe_time_buffer"].asFloat();
        planning_conf_.speed_optimizer_conf.speed_range_buffer =
            root["speed_optimizer_conf"]["speed_range_buffer"].asFloat();
        planning_conf_.speed_optimizer_conf.safe_distance =
            root["speed_optimizer_conf"]["safe_distance"].asFloat();
        planning_conf_.speed_optimizer_conf.obstacle_weight =
            root["speed_optimizer_conf"]["obstacle_weight"].asFloat();
        planning_conf_.speed_optimizer_conf.speed_weight =
            root["speed_optimizer_conf"]["speed_weight"].asFloat();
        planning_conf_.speed_optimizer_conf.accel_weight =
            root["speed_optimizer_conf"]["accel_weight"].asFloat();
        planning_conf_.speed_optimizer_conf.jerk_weight =
            root["speed_optimizer_conf"]["jerk_weight"].asFloat();
        planning_conf_.speed_optimizer_conf.speed_limit =
            root["speed_optimizer_conf"]["speed_limit"].asFloat();
        // qp
        planning_conf_.speed_optimizer_conf.qp_opt_num =
            root["speed_optimizer_conf"]["qp_opt_num"].asInt();
        planning_conf_.speed_optimizer_conf.qp_seg_num =
            root["speed_optimizer_conf"]["qp_seg_num"].asInt();
        planning_conf_.speed_optimizer_conf.qp_param_num =
            root["speed_optimizer_conf"]["qp_param_num"].asInt();
        planning_conf_.speed_optimizer_conf.qp_inequ_num =
            root["speed_optimizer_conf"]["qp_inequ_num"].asInt();
        planning_conf_.speed_optimizer_conf.st_points_num =
            root["speed_optimizer_conf"]["st_points_num"].asInt();
        planning_conf_.speed_optimizer_conf.normal_jerk =
            root["speed_optimizer_conf"]["normal_jerk"].asDouble();
        planning_conf_.speed_optimizer_conf.upper_boundary =
            root["speed_optimizer_conf"]["upper_boundary"].asDouble();
        planning_conf_.speed_optimizer_conf.lower_boundary =
            root["speed_optimizer_conf"]["lower_boundary"].asDouble();
        planning_conf_.speed_optimizer_conf.qp_weight_1 =
            root["speed_optimizer_conf"]["qp_weight_1"].asDouble();
        planning_conf_.speed_optimizer_conf.qp_weight_2 =
            root["speed_optimizer_conf"]["qp_weight_2"].asDouble();
        planning_conf_.speed_optimizer_conf.qp_weight_3 =
            root["speed_optimizer_conf"]["qp_weight_3"].asDouble();
        planning_conf_.speed_optimizer_conf.qp_weight_4 =
            root["speed_optimizer_conf"]["qp_weight_4"].asDouble();
        // traffic_light_conf
        planning_conf_.traffic_light_conf.use_keyboard =
            root["traffic_light_conf"]["use_keyboard"].asBool();
        planning_conf_.traffic_light_conf.virtual_wall_by_perception =
            root["traffic_light_conf"]["virtual_wall_by_perception"].asBool();
        planning_conf_.traffic_light_conf.remaining_time_limit =
            root["traffic_light_conf"]["remaining_time_limit"].asInt();
        planning_conf_.traffic_light_conf.virtual_wall_length =
            root["traffic_light_conf"]["virtual_wall_length"].asDouble();
        planning_conf_.traffic_light_conf.virtual_wall_width =
            root["traffic_light_conf"]["virtual_wall_width"].asDouble();
        planning_conf_.traffic_light_conf.virtual_wall_height =
            root["traffic_light_conf"]["virtual_wall_height"].asDouble();
    }

    return true;
}
}  // namespace planning_lib
}  // namespace jarvis