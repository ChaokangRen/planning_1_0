#include "lane_borrow_decider.h"

namespace jarvis {
namespace planning_lib {

void LaneBorrowDecider::Process(const std::vector<Obstacle> &obstacles,
                                const PosFromIns &pos_ins,
                                const SpeedFromVeh &speed_veh,
                                const ReferenceLine &refline) {
    bool has_left_nudge = false;
    bool has_right_nudge = false;

    for (const Obstacle &obs : obstacles) {
        if (obs.Behavior() == Obstacle::BEHAVIORTYPE::LEFTNUDGE) {
            has_left_nudge = true;
        }
        if (obs.Behavior() == Obstacle::BEHAVIORTYPE::RIGHTNUDGE) {
            has_right_nudge = true;
        }

        if (has_left_nudge && has_right_nudge) {
            break;
        }
    }

    if (has_left_nudge) {
        ++count_left_;
    } else {
        count_left_ = 0;
    }

    if (has_right_nudge) {
        ++count_right_;
    } else {
        count_right_ = 0;
    }

    if (count_left_ > 20 && count_right_ <= 20) {
        need_lane_borrow_left_ = true;
        need_lane_borrow_right_ = false;
        decide_status_ = DecideStatus::LANE_BORROW_LEFT;
        lb_ubound_ = 3;
        lb_lbound_ = -0.5;
    } else if (count_right_ > 20 && count_left_ <= 20) {
        need_lane_borrow_left_ = false;
        need_lane_borrow_right_ = true;
        decide_status_ = DecideStatus::LANE_BORROW_RIGHT;
        lb_ubound_ = 0.5;
        lb_lbound_ = -3;
    } else {
        need_lane_borrow_left_ = false;
        need_lane_borrow_right_ = false;
        decide_status_ = DecideStatus::LANE_FOLLOW;
        lb_ubound_ = 0.5;
        lb_lbound_ = -0.5;
    }
}

// void LaneBorrowDecider::Process(const std::vector<Obstacle> &obstacles,
//                                 const PosFromIns &pos_ins,
//                                 const SpeedFromVeh &speed_veh,
//                                 const ReferenceLine &refline) {
//     if (HasPreStaticObs(obstacles, pos_ins, refline) ||
//         HasStaticObs(obstacles, pos_ins, refline)) {
//         count_stop_++;
//     } else {
//         cipv_id_.clear();
//         count_stop_ = 0;
//     }

//     if (!cipv_id_.empty()) {
//         SG_INFO("%s -- borrow", cipv_id_.c_str());
//     } else {
//         SG_INFO("no borrow");
//     }

//     SG_INFO("count_stop = %d", count_stop_);
//     if (count_stop_ > 10) {
//         SG_WARN("LANE_BORROW");
//         decide_status_ = DecideStatus::LANE_BORROW;
//         need_lane_borrow_ = true;
//         lb_ubound_ = 3;
//         lb_lbound_ = -3;
//     } else {
//         decide_status_ = DecideStatus::LANE_FOLLOW;
//         need_lane_borrow_ = false;
//         lb_ubound_ = 0.5;
//         lb_lbound_ = -0.5;
//     }
// }

// bool LaneBorrowDecider::HasPreStaticObs(const std::vector<Obstacle>
// &obstacles,
//                                         const PosFromIns &pos_ins,
//                                         const ReferenceLine &refline) {
//     for (const auto &obs : obstacles) {
//         std::string id = obs.Id();
//         if (id == cipv_id_) {
//             return IsCloseStaticObs(obs, pos_ins, refline);
//         }
//     }
//     return false;
// }

// bool LaneBorrowDecider::HasStaticObs(const std::vector<Obstacle> &obstacles,
//                                      const PosFromIns &pos_ins,
//                                      const ReferenceLine &refline) {
//     for (const auto &obs : obstacles) {
//         if (IsCloseStaticObs(obs, pos_ins, refline)) {
//             cipv_id_ = obs.Id();
//             return true;
//         };
//     }
//     return false;
// }

// bool LaneBorrowDecider::IsCloseStaticObs(const Obstacle &obs,
//                                          const PosFromIns &pos_ins,
//                                          const ReferenceLine &refline) {
//     if (obs.Speed() > 0.5) {
//         // SG_INFO("%s -- move", obs.Id().c_str());
//         return false;
//     }
//     if (obs.Id() == "virtual wall obstacle") {
//         // SG_INFO("%s -- wall", obs.Id().c_str());
//         return false;
//     }

//     double obs_start_s = std::numeric_limits<double>::max();
//     double obs_end_s = -std::numeric_limits<double>::max();
//     double obs_start_l = std::numeric_limits<double>::max();
//     double obs_end_l = -std::numeric_limits<double>::max();

//     for (const auto &obs_point : obs.GetObstacleBox().GetAllCorners()) {
//         // SG_INFO("obs_x =%f,obs_y =%f", obs_point.x(), obs_point.y());
//         double pos_s;
//         double pos_l;
//         // SG_INFO("%s -- obs_point.x()=%lf,obs_point.y()=%lf",
//         // obs.Id().c_str(),
//         //         obs_point.x(), obs_point.y());
//         refline.XyToSl(obs_point.x(), obs_point.y(), &pos_s, &pos_l);
//         // SG_INFO("%s -- pos_s=%lf,pos_l=%lf", obs.Id().c_str(), pos_s,
//         pos_l); obs_start_s = std::fmin(obs_start_s, pos_s); obs_end_s =
//         std::fmax(obs_end_s, pos_s); obs_start_l = std::fmin(obs_start_l,
//         pos_l); obs_end_l = std::fmax(obs_end_l, pos_l);
//     }
//     // SG_INFO("%s --
//     // obs_start_s=%lf,obs_end_s=%lf,obs_start_l=%lf,obs_end_l=%lf",
//     //         obs.Id().c_str(), obs_start_s, obs_end_s, obs_start_l,
//     //         obs_end_l);

//     if (obs_end_s < -5 || obs_start_s > 40) {
//         return false;
//     }

//     double dx = obs.Center().x() - pos_ins.position.x;
//     double dy = obs.Center().y() - pos_ins.position.y;
//     double dist = std::hypot(dx, dy);

//     if (dist > 40) {
//         return false;
//     }

//     double obs_near_l = std::fmin(std::fabs(obs_end_l),
//     std::fabs(obs_start_l));

//     if (obs_near_l < 1.75) {
//         // SG_INFO("%s -- obs_end_s=%lf,obs_near_l=%lf", obs.Id().c_str(),
//         //         obs_end_s, obs_near_l);
//         return true;
//     }
//     return false;
// }
}  // namespace planning_lib
}  // namespace jarvis