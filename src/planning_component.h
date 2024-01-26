#pragma once

#include <map>
#include <memory>

#include "common.h"
#include "json_parser.h"
#include "lane.h"
#include "local_path_generator.h"
#include "obstacle.h"
#include "obstacle_container.h"
#include "planner_interface.h"
#include "planning_interface.h"
#include "prediction/obstacle_prediction.h"
#include "reference_line_smooth/reference_line_smooth.h"
#include "routing.h"
#include "rtk_generator/rtk_generator.h"
#include "traffic_light.h"

namespace jarvis {
namespace planning_lib {

typedef std::shared_ptr<jarvis::planning_lib::PlannerInterface>
    PlannerInterfacePtr;

class PlanningComponent : public PlanningInterface {
public:
    PlanningComponent();
    virtual ~PlanningComponent();

    virtual std::string get_version() override;

    // virtual bool Init(const std::string &rtk_path =
    //                       "/usr/share/jarvis/planning/resource/rtk_trajectory/"
    //                       "qidi_city_demo_fd.json") override;
    // virtual bool Execute(const LaneInfo &lane_info,
    //                      const VehicleState &vehicle_state,
    //                      const ObstaclesInfo &obstacles_msg,
    //                      const TrafficLightFrameResult
    //                      &traffic_lights_results, TrajectoryMsg
    //                      *trajectory_msg, VirtualWallsMsg *virtual_walls_msg,
    //                      std::string &debuginfo) override;

    virtual bool init(const PlanningConfigPath &planning_config_path,
                      const std::vector<PosFromIns> &history_ins_pts) override;
    virtual bool execute(TrajectoryMsg *trajectory_msg,
                         VirtualWallsMsg *virtual_walls_msg,
                         std::string &debuginfo) override;

    virtual bool set_pos_from_ins(const PosFromIns &pos_ins) override;
    virtual bool set_speed_from_veh(const SpeedFromVeh &speed_veh) override;
    virtual bool set_wheel_speed_from_veh(
        const WheelSpeedFromVeh &wheel_speed_veh) override;
    virtual bool set_obstacles_info(
        const ObstaclesInfo &obstacle_info) override;
    virtual bool set_lane_info(const LaneInfo &lane_info) override;
    virtual bool set_traffic_light_frame_result(
        const TrafficLightFrameResult &traffic_light_frame_result) override;

private:
    // Use the current line to shift out the new line
    std::vector<PathPoint> GetNewLineByReferenceLineShiftOut(
        const std::vector<LanePoint> &reference_lane_line, const double l);

    void GetLaneLinesOnSidesOfVehicle(const LaneInfo &lane_info,
                                      const PosFromIns &pos_ins, int32_t *left,
                                      int32_t *right);

    std::vector<PathPoint> GetNewLineByReferenceLineShiftOut(
        const std::vector<PathPoint> &reference_lane_line, const double l);

    void ObstacleMapInsert(const ObstaclesInfo &obstacle_info);

    bool ObstacleFilter(const ObstacleMsg &obstacle, const PosFromIns &pos_ins,
                        const SpeedFromVeh &speed_veh, bool &is_behind,
                        Vec2d &pos_in_veh);

    bool RearObstacleProcess(std::vector<Obstacle> &obstacles);

    void UpdateHistoryTrajectory(const PosFromIns &pos_ins);

private:
    JsonParser json_parser_;
    PlanningConf planning_conf_;
    TrafficLight traffic_light_;
    Lane lane_;

    PosFromIns pos_ins_;
    SpeedFromVeh speed_veh_;
    WheelSpeedFromVeh wheel_speed_veh_;
    ObstaclesInfo obstacle_info_;
    LaneInfo lane_info_;
    TrafficLightFrameResult traffic_light_frame_result_;

    ReferenceLineSmooth reference_line_smooth_;

    PlannerInterfacePtr planner_ptr_;

    std::pair<std::vector<PathPoint>, std::vector<PathPoint>> navigable_area_;

    std::vector<PathPoint> lane_center_line_;

    double lane_width_ = 2.7;  // 6;  // 2.7;  // 3.75;

    double epsilon_ = 1e-3;

    double vehicle_length_ = 4.983;
    double vehicle_width_ = 1.93;

    RtkGenerator rtk_generator_;

    RtkTrajectoryMsg rtk_trajectory_msg_;

    bool has_dynamic_obs_ = false;

    int32_t lane_id_ = 0;

    double vel_desire_ = 0.0;
    double vel_past_ = 0.0;

    std::map<std::string, bool> obstacle_list_;

    double average_time_ = 0;
    double max_time_ = 0.0;
    int32_t time_counts_ = 0;

    double total_length_ = 0.0;
    double total_time_ = 0.0;
    double refline_length_ = 0.0;

    TrajectoryMsg trajectory_msg_pre_;

    std::list<Vec2d> history_trajectory_;

    LocalPathGenerator local_path_generator_;
    Routing routing_;
};

}  // namespace planning_lib
}  // namespace jarvis
