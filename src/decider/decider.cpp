#include "decider.h"

namespace jarvis {
namespace planning_lib {

void Decider::Init() {
    ubound_ = 1.75;
    lbound_ = -1.75;
    decide_status_ = DecideStatus::LANE_FOLLOW;
    SG_INFO("ubound =%f,lbound =%f,status = %d", ubound_, lbound_,
            decide_status_);
}

void Decider::Process(const Lane &lane, const std::vector<Obstacle> &obstacles,
                      const PosFromIns &pos_ins, const SpeedFromVeh &speed_veh,
                      const LaneChangeMsg &lane_change_msg,
                      const ReferenceLine &refline,
                      const DriveState &drive_state, std::string &debuginfo) {
    double dis2stopline = lane.GetDistance2StopLine();
    // SG_INFO("dis2stopline=%lf", dis2stopline);
    if (drive_state != DriveState::LaneFollow) {
        SG_WARN("not allowed to lane borrow!!!");
        decide_status_ = DecideStatus::LANE_FOLLOW;
    } else {
        lane_borrow_decider_.Process(obstacles, pos_ins, speed_veh, refline);
        decide_status_ = lane_borrow_decider_.GetDecideStatus();
        ubound_ = lane_borrow_decider_.GetLbUpBound();
        lbound_ = lane_borrow_decider_.GetLbLowBound();
    }
    UpdateBound(decide_status_);
    UpdateDebugInfo(decide_status_, debuginfo);
    PrintStatus();
    return;
}

void Decider::UpdateBound(const DecideStatus &status) {
    if (status == DecideStatus::LANE_FOLLOW) {
        ubound_ = 0.5;
        lbound_ = -0.5;
    } else {
        // ubound_ = 0.5;
        // lbound_ = -0.5;
    }
    return;
}

void Decider::reset() {
    ubound_ = 0.5;
    lbound_ = -0.5;
    decide_status_ = DecideStatus::LANE_FOLLOW;
}

void Decider::UpdateDebugInfo(const DecideStatus &status,
                              std::string &debuginfo) const {
    debuginfo += "{decide_status:";
    if (decide_status_ == DecideStatus::LANE_FOLLOW) {
        debuginfo += "LANE_FOLLOW";
    } else if (decide_status_ == DecideStatus::LANE_CHANGE_LEFT) {
        debuginfo += "LANE_CHANGE_LEFT";
    } else if (decide_status_ == DecideStatus::LANE_CHANGE_RIGHT) {
        debuginfo += "LANE_CHANGE_RIGHT";
    } else if (decide_status_ == DecideStatus::ROUTE_CHANGE_LEFT) {
        debuginfo += "ROUTE_CHANGE_LEFT";
    } else if (decide_status_ == DecideStatus::ROUTE_CHANGE_RIGHT) {
        debuginfo += "ROUTE_CHANGE_RIGHT";
    } else if (decide_status_ == DecideStatus::LANE_BORROW_LEFT) {
        debuginfo += "LANE_BORROW_LEFT";
    } else if (decide_status_ == DecideStatus::LANE_BORROW_RIGHT) {
        debuginfo += "LANE_BORROW_RIGHT";
    }
    debuginfo += "decide_status_end}";
}

void Decider::PrintStatus() const {
    if (decide_status_ == DecideStatus::LANE_BORROW_LEFT) {
        SG_INFO("DecideStatus::LANE_BORROW_LEFT");
    } else if (decide_status_ == DecideStatus::LANE_BORROW_RIGHT) {
        SG_INFO("DecideStatus::LANE_BORROW_RIGHT");
    } else if (decide_status_ == DecideStatus::LANE_CHANGE_LEFT) {
        SG_INFO("DecideStatus::LANE_CHANGE_LEFT");
    } else if (decide_status_ == DecideStatus::LANE_CHANGE_RIGHT) {
        SG_INFO("DecideStatus::LANE_CHANGE_RIGHT");
    } else if (decide_status_ == DecideStatus::LANE_FOLLOW) {
        SG_INFO("DecideStatus::LANE_FOLLOW");
    } else if (decide_status_ == DecideStatus::ROUTE_CHANGE_LEFT) {
        SG_INFO("DecideStatus::ROUTE_CHANGE_LEFT");
    } else if (decide_status_ == DecideStatus::ROUTE_CHANGE_RIGHT) {
        SG_INFO("DecideStatus::ROUTE_CHANGE_RIGHT");
    } else {
        SG_INFO("DecideStatus::???");
    }
}

}  // namespace planning_lib
}  // namespace jarvis