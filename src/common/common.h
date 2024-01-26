/**
 * @file
 * @brief Define planning data struct
 */
#pragma once
#include <stdint.h>

#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <vector>

#include "planning_interface.h"
#include "vec2d.h"

namespace jarvis {
namespace planning_lib {

struct SLPoint {
    double s;
    double l;
};

typedef struct {
    double s;
    double t;
    double velocity;
    double acc;
    double dot_acc;
} SpeedPoint;

typedef struct {
    double start_s = 0;
    double end_s = 0;
    double start_l = 0;
    double end_l = 0;
} SLStaticBoundary;

typedef struct {
    SLPoint pos;
    double dot_l;
    double ddot_l;
    double dddot_l;
    double theta = 0.0;
} DPPathPoint;
typedef std::vector<DPPathPoint> DpPathData;

struct RtkTrajectoryMsg {
    std::vector<PathPoint> rtk_trajectory;
    bool has_rtk_msg;
};

struct VehicleParams {
    double front_edge_to_center = 0.0;
    double back_edge_to_center = 0.0;
    double left_edge_to_center = 0.0;
    double right_dege_to_center = 0.0;
    double rear_view_mirror_width = 0.0;
    double vehicle_width = 0.0;
    double vehicle_length = 0.0;
    double min_turn_radius = 0.0;
    double max_acceleration = 0.0;
    double max_deceleartion = 0.0;
    double max_steer_angle = 0.0;
    double max_steer_angle_rate = 0.0;
    double min_steer_angle_rate = 0.0;
    double steer_ratio = 0.0;
    double wheel_base = 0.0;
    double mass = 0.0;
    double max_centric_acc_limit = 0.0;
    double minimal_kappa = 0.0;
};

struct ReferenceLineSmoothConf {
    double refline_total_length = 0.0;
    int32_t refline_total_num = 0;
    int32_t seg_num = 0;
    double interval = 0.0;
    double ref_line_cost_weight = 0.0;
    int32_t qp_param_num = 0;
    int32_t qp_inequ_num = 0;
    double qp_weight_1 = 0.0;
    double qp_weight_2 = 0.0;
    double qp_weight_3 = 0.0;
    double qp_weight_4 = 0.0;
};

struct PlannerConf {
    double total_length = 0.0;
    double total_time = 0.0;
    double trajectory_time_interval = 0.0;
};

struct PathOptimizerConf {
    int32_t lateral_sample_num = 0;
    int32_t longitudinal_sample_num = 0;
    double boundary_buffer = 0.0;
    double offset_to_other_referenceline = 0.0;
    double sige_pass_distance = 0.0;

    double eval_time_interval = 0.0;
    float dp_path_cost_sample_dist = 0.0;
    float dp_path_cost = 0.0;
    float dp_path_dl_cost = 0.0;
    float dp_path_ddl_cost = 0.0;
    float dp_static_obs_cost = 0.0;
    float dp_dynamic_obs_cost_small = 0.0;
    float dp_dynamic_obs_cost_big = 0.0;
    double has_collision_cost = 0.0;
    double safety_distance = 0.0;
    double obstacle_collision_distance = 0.0;
    double risk_obstacle_collision_distance = 0.0;
    double obstacle_ignore_distance = 0.0;
    double static_obs_safe_ratio = 0.0;

    int32_t lane_num = 0;
    int32_t qp_param_num = 0;
    int32_t qp_inequ_num = 0;
    double t_interval = 0.0;
    double longitudinal_safe_buffer = 0.0;
    double lateral_safe_buffer = 0.0;
    double qp_weight_1 = 0.0;
    double qp_weight_2 = 0.0;
    double qp_weight_3 = 0.0;
    double qp_weight_4 = 0.0;
};

struct SpeedOptimizerConf {
    int32_t dimension_s = 0;
    float unit_s = 0.0;
    float unit_t = 0.0;
    float safe_time_buffer = 0.0;
    float speed_range_buffer = 0.0;
    float safe_distance = 0.0;
    float obstacle_weight = 0.0;
    float speed_weight = 0.0;
    float accel_weight = 0.0;
    float jerk_weight = 0.0;
    float speed_limit = 0.0;

    int32_t qp_opt_num = 0;
    int32_t qp_seg_num = 0;
    int32_t qp_param_num = 0;
    int32_t qp_inequ_num = 0;
    int32_t st_points_num = 0;
    double normal_jerk = 0.0;
    double upper_boundary = 0.0;
    double lower_boundary = 0.0;
    double qp_weight_1 = 0.0;
    double qp_weight_2 = 0.0;
    double qp_weight_3 = 0.0;
    double qp_weight_4 = 0.0;
};

struct TrafficLightConf {
    bool use_keyboard = false;
    bool virtual_wall_by_perception = false;
    int remaining_time_limit = 0;
    double virtual_wall_length = 0.0;
    double virtual_wall_width = 0.0;
    double virtual_wall_height = 0.0;
};

struct PlanningConf {
    VehicleParams vehicle_params;
    ReferenceLineSmoothConf reference_line_smooth_conf;
    PlannerConf planner_conf;
    PathOptimizerConf path_optimizer_conf;
    SpeedOptimizerConf speed_optimizer_conf;
    TrafficLightConf traffic_light_conf;
};

struct Constraint {
    std::vector<double> lower_bound;
    std::vector<double> upper_bound;
    std::vector<std::vector<double>> constraint_mat;
};

class DebugInfo {
public:
    std::string GetDebugInfo() {
        return debug_info_;
    }
    void SetDebugInfo(const std::string &str) {
        debug_info_ += str;
    }

    void Clear() {
        debug_info_.clear();
    }

private:
    std::string debug_info_;
};

template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
    if (std::abs(t1 - t0) <= 1.0e-6) {
        return x0;
    }
    const double r = (t - t0) / (t1 - t0);
    const T x = x0 + r * (x1 - x0);
    return x;
}

Vec2d PointRotate(const Vec2d &point, const double x, const double y,
                  const double theta);

double LinearInterpolateOfTheta(const double a0, const double t0,
                                const double a1, const double t1,
                                const double t);

double NormalizeAngle(const double angle);

inline void LanePointsTransPathPoints(const std::vector<LanePoint> &lane,
                                      std::vector<PathPoint> *path) {
    path->clear();
    for (int i = 0; i < lane.size(); ++i) {
        path->emplace_back(lane[i].point);
    }
}

// Compute the cost function matrix  for first martix
Eigen::MatrixXd ComputeCostFunctionFristMatrix(
    const std::vector<double> &s_order);

// Compute the cost function matrix  for second martix
Eigen::MatrixXd ComputeCostFunctionSecondMatrix(
    const std::vector<double> &s_order);

// Compute the cost function matrix  for thrid martix
Eigen::MatrixXd ComputeCostFunctionThirdMatrix(
    const std::vector<double> &s_order);

// Compute the cost function matrix  for forth martix
Eigen::MatrixXd ComputeCostFunctionForthMatrix(
    const std::vector<double> &s_order);

typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef double DpCost;

bool IsNumeric(const std::string &str);

enum class StopLineStatus { None = 0, Front = 1, Rear = 2 };

enum class DecideStatus {
    LANE_FOLLOW = 0,
    LANE_CHANGE_LEFT = 1,
    LANE_CHANGE_RIGHT = 2,
    ROUTE_CHANGE_LEFT = 3,
    ROUTE_CHANGE_RIGHT = 4,
    LANE_BORROW_LEFT = 5,
    LANE_BORROW_RIGHT = 6
};

enum class TURN { LEFT = 0, STRAIGHT = 1, RIGHT = 2, NONE = 3 };
enum class ROADGRADE { PRIMARY = 0, SECONDARY, TERTIARY, RESIDENTIAL, NONE };
typedef struct {
    double x;
    double y;
    double yaw;
    double speed_limit;
    TURN turn;
} RoutingNode;

typedef struct {
    double distance = 0;
    TURN turn;
    double speed_limit_mps = 0;
    ROADGRADE grade = ROADGRADE::NONE;
    double next_road_theta = 0.0;
    double current_road_theta = 0.0;
    std::vector<Point3d> next_road_ref_points;

} RoutingMsg;

typedef struct {
    bool is_lane_change = false;
    TURN turn = TURN::NONE;
    std::string target_lane_id;
    std::string curr_lane_id;
} LaneChangeMsg;
constexpr double deg_to_rad = 0.017453292519943295;

enum class Source {
    MiddleValue = 0,
    LeftOffset = 1,
    RightOffset = 2,
    NONE = 3
};
struct FitCurve {
    // y = a0 + a1 * x^2 + ... + an * x^n
    std::vector<double> fit_param;
    Vec2d start;
    Vec2d end;

    std::string id;
    //-1-left, 0-middle ,1-right
    int32_t digit_id = -1;
    Source source;

    bool can_left_change = false;
    bool can_right_change = false;
};

void LeastSquareFitCurve(const std::vector<Vec2d> &points, const int32_t order,
                         FitCurve &fit_curve);

struct PnCLane {
    std::vector<FitCurve> lanes;
    std::vector<std::vector<PathPoint>> center_lines;
    int32_t self_car_lane_cnt = 0;
};

void TurnPrint(const TURN turn);

void PnCLaneAddStarightLine(PnCLane &pnc_lanes);

double AverageSpeed(const double v0, const double ref_v, const double a,
                    const double l);

}  // namespace planning_lib
}  // namespace jarvis