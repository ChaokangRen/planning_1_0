#pragma once
#include "common.h"
#include "planning_interface.h"
#include "virtual_obs.h"
namespace jarvis {
namespace planning_lib {

class Lane {
public:
    void Init();

    void Process(const LaneInfo &lane_info, const SpeedFromVeh &speed_veh,
                 const PosFromIns &pos_ins_,
                 const std::vector<PathPoint> &local_path);

    float GetDistance2StopLine() const {
        return current_veh2stopline_dist_;
    }

    bool HasStopLine() const {
        if (has_stop_line_) {
            if (current_veh2stopline_dist_ < 0) {
                return false;
            }
            return true;
        } else {
            if (current_veh2stopline_dist_ < 10 &&
                current_veh2stopline_dist_ > 0) {
                return true;
            }
        }
        return false;
    }
    void Reset(void);

    StopLineStatus Status(void) const {
        return stop_line_status_;
    }

private:
    void StopLineProcess(const LaneInfo &lane_info,
                         const std::vector<PathPoint> &local_path);

    void UpdateHasStopLine(const LaneInfo &lane_info,
                           const std::vector<PathPoint> &local_path);

    static bool Close2FarSortStopLineMsg(const LaneLineMsg &lane_line_msg_1,
                                         const LaneLineMsg &lane_line_msg_2);

    void StopLineFilter(const double current_dist, const LaneInfo &lane_info);

    double UpdateStopLineList(const double stop_line_dist);

    void UpdateVehicleMoveDistance(const LaneInfo &lane_info);

    void Variance(void);

    bool IsStopLineOnLocalPath(const LineSegment2d &stop_line,
                               const std::vector<PathPoint> &local_path);

private:
    LaneInfo lane_info_;
    SpeedFromVeh speed_veh_;

    std::vector<LaneLineMsg> stop_lines_;

    bool has_stop_line_ = false;

    bool has_stop_line_pre_ = false;

    float distance_to_stop_line_ = -1;

    double distance_to_stop_line_limit_ = 50.0;

    std::list<double> stop_line_list_;

    bool pre_has_stop_line_ = false;
    int16_t stop_line_count_ = 0;
    int16_t hasnot_stop_line_count_ = 0;

    double vehicle_move_distance_ = 0.0;
    double observe_stop_line_dist_ = 0;

    Vec2d vehicle_pos_pre;

    int16_t list_max_cnt_ = 20;

    bool is_actiove_move_ = false;

    double current_veh2stopline_dist_ = 999.0;

    double variance_ = -1;

    StopLineStatus stop_line_status_ = StopLineStatus::None;

    int16_t stop_line_cold_time_ = 0;

    double sin_trans_theta_ = 0;
    double cos_trans_theta_ = 0;
    double veh_x_ = 0.0;
    double veh_y_ = 0.0;
};

}  // namespace planning_lib
}  // namespace jarvis