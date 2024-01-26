#pragma once

#include <algorithm>
#include <memory>

#include "common.h"
#include "obstacle.h"
#include "reference_line.h"
#include "traffic_light.h"

namespace jarvis {
namespace planning_lib {

class LaneBorrowDecider {
public:
    LaneBorrowDecider() = default;

    void Process(const std::vector<Obstacle> &obstacles,
                 const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
                 const ReferenceLine &refline);

    double GetLbUpBound() const {
        return lb_ubound_;
    }

    double GetLbLowBound() const {
        return lb_lbound_;
    }

    DecideStatus GetDecideStatus() const {
        return decide_status_;
    }

private:
    bool HasPreStaticObs(const std::vector<Obstacle> &obstacles,
                         const PosFromIns &pos_ins,
                         const ReferenceLine &refline);
    bool HasStaticObs(const std::vector<Obstacle> &obstacles,
                      const PosFromIns &pos_ins, const ReferenceLine &refline);
    bool IsCloseStaticObs(const Obstacle &obs, const PosFromIns &pos_ins,
                          const ReferenceLine &refline);

    DecideStatus decide_status_ = DecideStatus::LANE_FOLLOW;

    double lb_ubound_;
    double lb_lbound_;

    int count_left_ = 0;
    int count_right_ = 0;

    bool need_lane_borrow_left_ = false;
    bool need_lane_borrow_right_ = false;
    std::string cipv_id_;
};

}  // namespace planning_lib
}  // namespace jarvis