#pragma once

#include <algorithm>
#include <memory>

#include "common.h"
#include "lane_borrow_decider.h"
#include "local_path_generator.h"
#include "obstacle.h"
#include "reference_line.h"
#include "traffic_light.h"

namespace jarvis {
namespace planning_lib {

class Decider {
public:
    Decider() = default;

    void Init();

    void Process(const Lane &lane, const std::vector<Obstacle> &obstacles,
                 const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
                 const LaneChangeMsg &lane_change_msg,
                 const ReferenceLine &refline, const DriveState &drive_state,
                 std::string &debuginfo);

    DecideStatus GetDecideStatus() const {
        return decide_status_;
    }

    double GetUpBound() const {
        return ubound_;
    }

    double GetLowBound() const {
        return lbound_;
    }

    void SetDecideStatus(const DecideStatus &status) {
        decide_status_ = status;
    }

    void PrintStatus() const;

private:
    void UpdateBound(const DecideStatus &status);
    void reset();
    void UpdateDebugInfo(const DecideStatus &status,
                         std::string &debuginfo) const;

private:
    LaneBorrowDecider lane_borrow_decider_;

    double ubound_;
    double lbound_;
    DecideStatus decide_status_ = DecideStatus::LANE_FOLLOW;
};
}  // namespace planning_lib
}  // namespace jarvis