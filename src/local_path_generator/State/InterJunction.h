#pragma once
#include <map>

#include "common.h"
#include "local_path_common.h"
#include "planning_interface.h"
namespace jarvis {
namespace planning_lib {

class InterJunctionState {
public:
    InterJunctionState();
    bool Process(const LaneInfo &lane_info, const PosFromIns &vehicle_state,
                 const ObstaclesInfo &obstacles_msg, const PnCLane &pnc_lanes,
                 const RoutingMsg &routing_msg, const Injector &injector,
                 std::vector<Point3d> &curvature_line);
    void Reset(void);

    std::vector<Point3d> GetExtendLine(void) {
        return lateral_extended_line_;
    }

private:
    void GetCenterLine(const PnCLane &pnc_lanes,
                       const PosFromIns &vehicle_state, const double length,
                       std::vector<Point3d> &curvature_line, double &last_x);

    void GetCenterLine(const LaneInfo &lane_info,
                       const PosFromIns &vehicle_state, const double length,
                       std::vector<Point3d> &curvature_line, double &last_x);

    void RoadWidthEstimateByVehicle(const PosFromIns &vehicle_state,
                                    const ObstaclesInfo &obstacles_msg);

    void LeftObsPriorityInsert(const double lon, const double lat,
                               const double vx, const double vy,
                               const double yaw);
    double ObsLateralDistEastimate(
        const std::vector<std::map<int32_t, int32_t>> &obs_priority_set);

    void RightObsPriorityInsert(const double lon, const double lat,
                                const double vx, const double vy,
                                const double yaw);

    void CalulateCurvatureLine(const LaneInfo &lane_info,
                               const PnCLane &pnc_lanes,
                               const PosFromIns &vehicle_state,
                               std::vector<Point3d> &curvature_line);

    void BezierCurveEstimate(const Point3d &p1, const Point3d &p2,
                             const Point3d &p3, const PosFromIns &vehicle_state,
                             std::vector<Point3d> &curvature_line);

    void CalulateStraightLine(const LaneInfo &lane_info,
                              const PnCLane &pnc_lanes,
                              const PosFromIns &vehicle_state,
                              std::vector<Point3d> &curvature_line);

    void NextRoadPointEstimateUseTraffic(Point3d &p2, Point3d &p3);

    void NextRoadPointEstimateUseRouteMap(const PosFromIns &vehicle_state,
                                          Point3d &p2, Point3d &p3);

    void NextRoadPointEstimateUseObstacle(Point3d &p2, Point3d &p3);

private:
    double cos_trans_theta_ = 0.0;
    double sin_trans_theta_ = 0.0;
    double stop_line_dist_eastimate_ = 0.0;
    double traffic_light_dist_ = 0.0;

    double turn_dist_ = 0.0;
    bool has_available_obstacle_ = false;

    double max_tolerance_rad_ = 25 * M_PI / 180;

    std::vector<std::map<int32_t, int32_t>> left_obs_priority_set_;
    std::vector<std::map<int32_t, int32_t>> right_obs_priority_set_;

    bool has_stable_slope_ = false;

    double left_road_width_estimate_ = 0.0;

    double lateral_road_width_ = 0.0;

    std::vector<Point3d> lateral_extended_line_;

    double additional_deg_ = 0.0;

    double right_turn_lat_dist_ = 0.0;
    double right_turn_lon_dist_ = 0.0;

    TURN turn_ = TURN::NONE;
    RoutingMsg routing_msg_;

    double lane_length_ = 90.1;
};
}  // namespace planning_lib
}  // namespace jarvis