#include <sglog/sglog.h>
#include <sgtime/sgtime.h>

#include "cartesian_frenet_conversion.h"
#include "planning/planning_interface.h"
#include "planning_component.h"
#include "reference_line.h"
#include "virtual_obs.h"

namespace jarvis {
namespace planning_lib {

PlanningInterface *PlanningInterface::create_instance() {
    return new PlanningComponent();
}

PlanningComponent::PlanningComponent() {
    SG_INFO("PlanningComponent construct");
}

PlanningComponent::~PlanningComponent() {
    SG_INFO("PlanningComponent destruct");
}

std::string PlanningComponent::get_version() {
#ifdef PKG_VERSION
    return PKG_VERSION;
#else
    return "UNKNOWN";
#endif
}

bool PlanningComponent::init(const PlanningConfigPath &planning_config_path,
                             const std::vector<PosFromIns> &history_ins_pts) {
    SG_INFO("PlanningComponent init");
    std::string config_path =
        "/usr/share/jarvis/planning/resource/planning_conf/planning_conf.json";
    json_parser_.ParserPlanningConf(config_path);

    planning_conf_ = json_parser_.GetPlanningConf();

    traffic_light_.Init(planning_conf_.traffic_light_conf);

    reference_line_smooth_.Init(planning_conf_.reference_line_smooth_conf);

    planner_ptr_.reset(
        jarvis::planning_lib::PlannerInterface::CreateInstance());
    planner_ptr_->Init(planning_conf_);
    std::string rtk_trajectory_path =
        "/usr/share/jarvis/planning/resource/rtk_trajectory/"
        "qidi_city_demo_3_7.json";
    // "bizhang.json";
    // "/usr/share/jarvis/planning/resource/rtk_trajectory/"
    // "test.json";
    // "college.json";

    if (!planning_config_path.rtk_path.empty()) {
        rtk_trajectory_path = planning_config_path.rtk_path;
    }

    refline_length_ =
        planning_conf_.reference_line_smooth_conf.refline_total_length;
    total_length_ = planning_conf_.planner_conf.total_length;
    total_time_ = planning_conf_.planner_conf.total_time;

    rtk_generator_.Init(rtk_trajectory_path, refline_length_);
    return true;
}

bool PlanningComponent::execute(TrajectoryMsg *trajectory_msg,
                                VirtualWallsMsg *virtual_walls_msg,
                                std::string &debuginfo) {
    auto begin = SgTimeUtil::now_nsecs();
    std::vector<double> times;
    ObstacleContainer obs_container(obstacle_info_);
    ObstaclesInfo obstacle_info_enu = obs_container.GetObsEnu();
    UpdateHistoryTrajectory(pos_ins_);

    double t1 = SgTimeUtil::now_secs();
    times.emplace_back(t1);

    LaneInfo lane_info;
    // lane_info.has_laneline_msg = false;

    pos_ins_.yaw = pos_ins_.yaw * M_PI / 180 + M_PI_2;
    // SG_INFO("pos_ins_x=%f,pos_ins_y=%f", pos_ins_.position.x,
    //         pos_ins_.position.y);

    // Get Local Path From Centerline
    std::vector<PathPoint> local_path;
    RoutingMsg routing_msg;
    LaneChangeMsg lane_change_msg;
    PnCLane pnc_center_lines;

    routing_.GetNextJunctionMsg(pos_ins_, routing_msg);
    DecideStatus decide_status = DecideStatus::LANE_FOLLOW;

    local_path_generator_.Process(
        lane_info_, pos_ins_, obstacle_info_enu, traffic_light_frame_result_,
        routing_msg, decide_status, pnc_center_lines, lane_change_msg);

    DriveState ds = local_path_generator_.GetDriveState();
    if (ds == DriveState::LaneFollow) {
        SG_INFO("LaneFollow");
    } else if (ds == DriveState::InterJunction) {
        SG_INFO("InterJunction");
    } else if (ds == DriveState::OnJunction) {
        SG_INFO("OnJunction");
    } else if (ds == DriveState::AwayJunction) {
        SG_INFO("AwayJunction");
    }

    const int32_t self_cnt = pnc_center_lines.self_car_lane_cnt;
    const auto &center_line = pnc_center_lines.center_lines;
    // SG_INFO("selfcnt = %d", self_cnt);

    // 1. traffic light msg
    // TrafficLightBox traffic_box =
    //     traffic_light_test_.TrafficLightFrameResultProcess(
    //         traffic_light_frame_result_, pos_ins_);
    traffic_light_.Process(traffic_light_frame_result_);

    //  DebugInfo debug_info;
    bool is_end_of_road = rtk_generator_.UpdateLocalPathForPlanning(pos_ins_);

    rtk_trajectory_msg_.rtk_trajectory = rtk_generator_.LocalPath(lane_id_);

    // 2. lane line msg
    lane_.Process(lane_info_, speed_veh_, pos_ins_,
                  rtk_trajectory_msg_.rtk_trajectory);

    rtk_trajectory_msg_.has_rtk_msg = true;
    // 1. Generator lane line and navigable area
    if (rtk_trajectory_msg_.has_rtk_msg == false &&
        lane_info.lane_lines_msg.empty()) {
        SG_ERROR("RTK data and lane line data are not available");
        return false;
    }
    std::vector<PathPoint> left_navigable_area;
    std::vector<PathPoint> right_navigable_area;
    // 2.If RTK msg has data and Laneline msg has no data
    if (rtk_trajectory_msg_.has_rtk_msg == true &&
        lane_info.lane_lines_msg.empty()) {
        lane_center_line_ = rtk_trajectory_msg_.rtk_trajectory;
        double left_boundary = 0.0;
        double right_boundary = 0.0;
        rtk_generator_.GetLaneBoundary(lane_id_, &left_boundary,
                                       &right_boundary);
        // SG_INFO("left_boundary = %lf,right_boundary = %lf", left_boundary,
        //         right_boundary);
        // Generator left navigable area
        left_navigable_area = std::move(GetNewLineByReferenceLineShiftOut(
            lane_center_line_, left_boundary));
        // Generator right navigable area
        right_navigable_area = std::move(GetNewLineByReferenceLineShiftOut(
            lane_center_line_, right_boundary));
    }
    // 3.If laneline msg has data
    if (!lane_info.lane_lines_msg.empty()) {
        SG_ERROR("current not support lane line version !");
        return false;
    }
    double t2 = SgTimeUtil::now_secs();
    times.emplace_back(t2);
    // 4. Reference line smooth
    std::vector<std::pair<QuinticPolynomialCurve, QuinticPolynomialCurve>>
        reference_line_curve =
            reference_line_smooth_.Smooth(lane_center_line_, pos_ins_);
    // for (int32_t i = 0; i < lane_center_line_.size(); ++i) {
    //     SG_INFO("ref x = %lf,y = %lf", lane_center_line_[i].position_m.x,
    //             lane_center_line_[i].position_m.y);
    // }
    // for (int32_t i = 0; i < reference_line_curve.size(); ++i) {
    //     for (float s = 0; s < 1.0; s += 0.05) {
    //         SG_INFO("smooth x = %lf,y = %lf",
    //                 reference_line_curve[i].first.CalcuteCurveValue(s),
    //                 reference_line_curve[i].second.CalcuteCurveValue(s));
    //     }
    // }
    // 5. Initialize the ReferenceLine class
    double t3 = SgTimeUtil::now_secs();
    times.emplace_back(t3);
    ReferenceLine reference_line(
        reference_line_curve, reference_line_smooth_.ReferenceLineCost(),
        rtk_generator_.LocalCenterLine(), refline_length_, total_time_);
    // auto lines = reference_line.CenterLines();
    // auto line = lines[0].GetLine();
    // for (int i = 0; i < line.size(); ++i) {
    //     SG_INFO("s=%lf,l=%lf", line[i].s, line[i].l);
    // }
    double tn = SgTimeUtil::now_secs();
    times.emplace_back(tn);
    std::vector<ReferenceLineInfo> reference_lines;
    reference_lines.emplace_back(ReferenceLineInfo{
        reference_line, left_navigable_area, right_navigable_area});
    double t4 = SgTimeUtil::now_secs();
    times.emplace_back(t4);
    // 6. Initialize the Obstacle class
    std::vector<Obstacle> obstacles;

    std::string obs_str = "{obstacles:";

    for (const ObstacleMsg &obstacle_msg : obstacle_info_enu.obstacles) {
        ObstacleMsg obstacle_msg_tmp = obstacle_msg;
        bool is_behind = false;
        Vec2d pos_in_veh;
        if (ObstacleFilter(obstacle_msg, pos_ins_, speed_veh_, is_behind,
                           pos_in_veh) == false) {
            continue;
        }
        if (is_behind == true || obstacle_msg_tmp.type != ObsType::VEHICLE) {
            for (int32_t i = 0; i < obstacle_msg_tmp.obs_trajectories.size();
                 ++i) {
                std::vector<TrajectoryPoint> points;
                for (const TrajectoryPoint &point :
                     obstacle_msg_tmp.obs_trajectories[i].points) {
                    points.emplace_back(point);
                    if (point.relative_time_s > 2.0) {
                        break;
                    }
                }
                obstacle_msg_tmp.obs_trajectories[i].points = points;
            }
        }

        if (obstacle_msg_tmp.obs_trajectories.empty()) {
            if (speed_veh_.speed_mps > 0) {
                ObsTrajectory trajectory;
                TrajectoryPoint point;
                point.path_point.position_m.x = obstacle_msg.state.position.x;
                point.path_point.position_m.y = obstacle_msg.state.position.y;
                point.path_point.theta_rad = obstacle_msg.state.angular_deg.z;
                point.path_point.kappa = 0.0;
                point.path_point.dkappa = 0.0;
                point.path_point.s_m = 0.0;
                for (float t = 0; t <= 6; t += 0.1) {
                    point.relative_time_s = t;
                    trajectory.points.emplace_back(point);
                }
                obstacle_msg_tmp.obs_trajectories.emplace_back(trajectory);
                // SG_INFO("obstacle_msg_tmp get trajecitory");
            }
        }

        obstacles.emplace_back(Obstacle(obstacle_msg_tmp));
        obstacles.back().SetPosInVeh(pos_in_veh);

        obstacles.back().SetSlBoundarys(reference_line.GetSlBoundary(
            obstacles.back().GetObstaclePolygon()));

        obs_str += "(x:" + std::to_string(obstacle_msg.state.position.x) + ",";
        obs_str += "y:" + std::to_string(obstacle_msg.state.position.y) + ",";
        obs_str +=
            "yaw:" + std::to_string(obstacle_msg.state.angular_deg.z) + ",";
        obs_str += "length:" + std::to_string(obstacle_msg.length) + ",";
        obs_str += "width:" + std::to_string(obstacle_msg.width);
        obs_str += "),";
    }
    // SG_WARN("pre obs = %d,filter obs = %d,percent = %lf",
    //         obstacle_info_enu.obstacles.size(), obstacles.size(),
    //         1.0 * obstacles.size() / obstacle_info_enu.obstacles.size());

    obs_str += "obstacles_end}";
    // debug_info.SetDebugInfo(obs_str);

    debuginfo += obs_str;
    double t5 = SgTimeUtil::now_secs();
    times.emplace_back(t5);
    // 6.2 process traffic light

    // if (traffic_box.is_enable == true) {
    //     bool has_virtual_obs = false;

    //     VirtualObs virtual_obstacle = traffic_light_process_.Process(
    //         reference_line, pos_ins_, traffic_box);

    //     if (virtual_obstacle.IsActived() == true) {
    //         obstacles.emplace_back(virtual_obstacle.GetObstacleMsg());
    //         obstacles.back().SetSlBoundarys(reference_line.GetSlBoundary(
    //             obstacles.back().GetObstaclePolygon()));
    //         virtual_walls_msg->emplace_back(virtual_obstacle.GetVirtualWall());
    //     }
    // }

    traffic_light_.VirtualWallProcess(lane_, pos_ins_, reference_line,
                                      virtual_walls_msg, obstacles);

    VirtualObs end_virtual_obs = rtk_generator_.EndVirtualObs(pos_ins_);
    if (end_virtual_obs.IsActived() == true) {
        obstacles.emplace_back(end_virtual_obs.GetObstacleMsg());
        obstacles.back().SetSlBoundarys(reference_line.GetSlBoundary(
            obstacles.back().GetObstaclePolygon()));
        virtual_walls_msg->emplace_back(end_virtual_obs.GetVirtualWall());
    }
    // 7. Execute emplanner module
    navigable_area_ = std::make_pair(left_navigable_area, right_navigable_area);
    trajectory_msg->timestamp = SgTimeUtil::now_secs();
    double t6 = SgTimeUtil::now_secs();
    times.emplace_back(t6);

    planner_ptr_->Execute(lane_, reference_lines, pos_ins_, speed_veh_,
                          lane_change_msg, pnc_center_lines,
                          local_path_generator_.GetDriveState(), trajectory_msg,
                          obstacles, debuginfo);

    double t7 = SgTimeUtil::now_secs();
    lane_id_ = planner_ptr_->GetCurrLaneId();
    times.emplace_back(t7);
    // // 8. update trajectory msg theta
    double theta_curr = 0, theta_pre = 0;

    for (int i = 0; i < trajectory_msg->trajectory.size(); ++i) {
        if (i == 0) {
            theta_curr = std::atan2(
                (trajectory_msg->trajectory[i + 1].path_point.position_m.y -
                 trajectory_msg->trajectory[i].path_point.position_m.y),
                (trajectory_msg->trajectory[i + 1].path_point.position_m.x -
                 trajectory_msg->trajectory[i].path_point.position_m.x));
            if (std::fabs(theta_curr - pos_ins_.yaw) > M_PI) {
                if (theta_curr - pos_ins_.yaw > M_PI) {
                    trajectory_msg->trajectory[i].path_point.theta_rad =
                        theta_curr - M_PI * 2;
                }
                if (pos_ins_.yaw - theta_curr > M_PI) {
                    trajectory_msg->trajectory[i].path_point.theta_rad =
                        theta_curr + M_PI * 2;
                }

            } else {
                trajectory_msg->trajectory[i].path_point.theta_rad = theta_curr;
            }

            if (std::fabs(trajectory_msg->trajectory[0].velocity_mps) < 0.2) {
                trajectory_msg->trajectory[i].path_point.theta_rad =
                    pos_ins_.yaw;
            }

        } else if (i > 0 && i < trajectory_msg->trajectory.size() - 1) {
            theta_curr = std::atan2(
                (trajectory_msg->trajectory[i + 1].path_point.position_m.y -
                 trajectory_msg->trajectory[i].path_point.position_m.y),
                (trajectory_msg->trajectory[i + 1].path_point.position_m.x -
                 trajectory_msg->trajectory[i].path_point.position_m.x));
            double delta_theta = theta_curr - theta_pre;
            if (std::fabs(delta_theta) > M_PI) {
                trajectory_msg->trajectory[i].path_point.theta_rad =
                    trajectory_msg->trajectory[i - 1].path_point.theta_rad +
                    (2 * M_PI - std::fabs(delta_theta));
            } else {
                trajectory_msg->trajectory[i].path_point.theta_rad =
                    trajectory_msg->trajectory[i - 1].path_point.theta_rad +
                    delta_theta;
            }
        } else {
            trajectory_msg->trajectory[i].path_point.theta_rad =
                trajectory_msg->trajectory[i - 1].path_point.theta_rad;
        }
        if (i != 0) {
            double ex =
                trajectory_msg->trajectory[i].path_point.position_m.x -
                trajectory_msg->trajectory[i - 1].path_point.position_m.x;
            double ey =
                trajectory_msg->trajectory[i].path_point.position_m.x -
                trajectory_msg->trajectory[i - 1].path_point.position_m.x;
        }
        theta_pre = theta_curr;
    }

    if (speed_veh_.speed_mps < 0.01 &&
        trajectory_msg->trajectory.front().acceleration < 0) {
        trajectory_msg->trajectory.front().acceleration = 0.0;
    }
    // debuginfo = debug_info.GetDebugInfo();
    // debug_info.Clear();

    double t8 = SgTimeUtil::now_secs();
    times.emplace_back(t8);
    double delta_t = t8 - t1;
    if (max_time_ < delta_t) {
        max_time_ = delta_t;
    }
    average_time_ += delta_t;
    time_counts_++;

    trajectory_msg_pre_ = *trajectory_msg;

    auto after = SgTimeUtil::now_nsecs();
    auto delta_summary =
        SgTimeUtil::time_delta_record(after, begin, "PlanningProcess");
    // average_delta_ns = 1117844, count = 226, lastest_delta_ns = 812606
    // for (int i = 1; i < times.size(); ++i) {
    //     SG_INFO("period%d_time = %lf", i, times[i] - times[i - 1]);
    // }
    // SG_INFO("PlanningProcess %s", delta_summary->to_str().c_str());
    return true;
}

std::vector<PathPoint> PlanningComponent::GetNewLineByReferenceLineShiftOut(
    const std::vector<LanePoint> &reference_lane_line, const double l) {
    std::vector<PathPoint> new_line;
    for (int32_t i = 0; i < reference_lane_line.size(); ++i) {
        new_line.emplace_back(reference_lane_line[i].point);
    }
    return GetNewLineByReferenceLineShiftOut(new_line, l);
}

std::vector<PathPoint> PlanningComponent::GetNewLineByReferenceLineShiftOut(
    const std::vector<PathPoint> &reference_lane_line, const double l) {
    std::vector<PathPoint> new_line = reference_lane_line;

    if (l >= -epsilon_ && l <= -epsilon_) {
        return new_line;
    } else {
        for (PathPoint &point : new_line) {
            double right_x = 0;
            double right_y = 0;
            CartesianFrenetConverter::FrentToCartesian(
                point.position_m.x, point.position_m.y, point.theta_rad,
                point.s_m, l, &right_x, &right_y);
            point.position_m.x = right_x;
            point.position_m.y = right_y;
        }
    }
    return new_line;
}

// Get lane lines on both sides of the vehicle
void PlanningComponent::GetLaneLinesOnSidesOfVehicle(const LaneInfo &lane_info,
                                                     const PosFromIns &pos_ins,
                                                     int32_t *left,
                                                     int32_t *right) {
    *left = *right = -1;
    if (lane_info.lane_lines_msg.empty()) {
        return;
    }
    Vec2d adc_point(pos_ins.position.x, pos_ins.position.y);

    int32_t lane_size = lane_info.lane_lines_msg.size();
    const std::vector<LaneLineMsg> &lanelines = lane_info.lane_lines_msg;
    for (int32_t i = 0; i < lane_size; ++i) {
        Vec2d line_point(lanelines[i].lane_line.front().point.position_m.x,
                         lanelines[i].lane_line.front().point.position_m.y);
        Vec2d line_point_rotate = PointRotate(line_point, pos_ins.position.x,
                                              pos_ins.position.y, pos_ins.yaw);

        if (line_point_rotate.y() + vehicle_width_ / 2 < 0) {
            *left = i;
            if (i + 1 < lane_size) {
                *right = i + 1;
            }
        } else if (line_point_rotate.y() + vehicle_width_ / 2 > 0 &&
                   line_point_rotate.y() - vehicle_width_ / 2 < 0) {
            if (lanelines[i].lane_line_type == LaneLineType::SOLID) {
                if (lane_size == 1 || i + 1 == lane_size) {
                    *left = -1;
                    *right = i;
                    break;
                } else {
                    *left = i;
                    *left = i + 1;
                    break;
                }
            } else if (lanelines[i].lane_line_type == LaneLineType::BROKEN) {
                if (pos_ins.yaw -
                        lanelines[i].lane_line.front().point.theta_rad >
                    0) {
                    *left = i - 1;
                    *right = i;
                    break;

                } else {
                    *left = i;
                    *right = i + 1;
                    break;
                }
            }
        } else if (line_point_rotate.y() - vehicle_width_ / 2 > 0) {
            if (*left >= 0) {
                break;
            } else {
                *left = -1;
                *right = i;
                break;
            }
        }
    }
    if (*right == lane_size) {
        *right = -1;
    }
}

void PlanningComponent::ObstacleMapInsert(const ObstaclesInfo &obstacle_info) {
    for (const ObstacleMsg &obs : obstacle_info.obstacles) {
        if (obstacle_list_.end() == obstacle_list_.find(obs.track_id)) {
            bool is_static = false;
            double speed = std::sqrt(
                obs.state.linear_velocity.x * obs.state.linear_velocity.x +
                obs.state.linear_velocity.y * obs.state.linear_velocity.y);
            if (speed < 0.2) {
                is_static = true;
            }
            obstacle_list_[obs.track_id] = is_static;
        }
    }
}

double obs_ignore_dist = 60;
double obs_ignore_speed = 16.67;
double obs_safe_dist = 25;
double response_time = 3;
bool PlanningComponent::ObstacleFilter(const ObstacleMsg &obstacle,
                                       const PosFromIns &pos_ins,
                                       const SpeedFromVeh &speed_veh,
                                       bool &is_behind, Vec2d &pos_in_veh) {
    double ex = obstacle.state.position.x - pos_ins.position.x;
    double ey = obstacle.state.position.y - pos_ins.position.y;
    double veh_yaw = pos_ins.yaw;
    double sin_yaw = std::sin(veh_yaw);
    double cos_yaw = std::cos(veh_yaw);
    double dx = ex * cos_yaw + ey * sin_yaw;
    double dy = -ex * sin_yaw + ey * cos_yaw;
    pos_in_veh.set_x(dx);
    pos_in_veh.set_y(dy);
    double obs_to_veh_dist = std::sqrt(ex * ex + ey * ey);

    double speed = std::sqrt(
        obstacle.state.linear_velocity.x * obstacle.state.linear_velocity.x +
        obstacle.state.linear_velocity.y * obstacle.state.linear_velocity.y);

    if (std::fabs(dy) > 20) {
        return false;
    }

    if (std::fabs(dy) > 5 && speed < 1.0) {
        return false;
    }

    if (obstacle.type != ObsType::VEHICLE) {
        return true;
    }

    if (obs_to_veh_dist > obs_ignore_dist && speed < obs_ignore_speed) {
        return false;
    }

    float radius_sqr = 6.25;
    // SG_INFO("dx = %lf,dy = %lf", dx, dy);
    is_behind = false;
    if (dx < 0 && dy < 20.0 && dy > -20.0) {
        for (const Vec2d pos : history_trajectory_) {
            double error_x = pos.x() - obstacle.state.position.x;
            double error_y = pos.y() - obstacle.state.position.y;
            if (error_x * error_x + error_y * error_y < radius_sqr) {
                return false;
            }
        }
        is_behind = true;
    }

    return true;
}

bool PlanningComponent::RearObstacleProcess(std::vector<Obstacle> &obstacles) {
    bool is_need_rear_obs = true;
    for (int32_t i = 0; i < obstacles.size(); ++i) {
        auto obs_sl = obstacles[i].GetObsSlBoundary();
        if (obs_sl.end_s > 0 && obs_sl.start_s > 0 && obs_sl.start_l > -5 &&
            obs_sl.start_l < 5) {
            is_need_rear_obs = false;
        }
    }
    if (is_need_rear_obs == false) {
        for (int32_t i = 0; i < obstacles.size(); ++i) {
            auto obs_sl = obstacles[i].GetObsSlBoundary();
            if (obs_sl.end_s < 0 && obs_sl.start_s < 0) {
                obstacles[i].SetStatic(true);
            }
        }
    }
    return is_need_rear_obs;
}

bool PlanningComponent::set_pos_from_ins(const PosFromIns &pos_ins) {
    pos_ins_ = pos_ins;
    return true;
}

bool PlanningComponent::set_speed_from_veh(const SpeedFromVeh &speed_veh) {
    speed_veh_ = speed_veh;
    return true;
}

bool PlanningComponent::set_wheel_speed_from_veh(
    const WheelSpeedFromVeh &wheel_speed_veh) {
    wheel_speed_veh_ = wheel_speed_veh;
    return true;
}

bool PlanningComponent::set_obstacles_info(const ObstaclesInfo &obstacle_info) {
    obstacle_info_ = obstacle_info;
    return true;
}
bool PlanningComponent::set_lane_info(const LaneInfo &lane_info) {
    lane_info_ = lane_info;
    return true;
}

bool PlanningComponent::set_traffic_light_frame_result(
    const TrafficLightFrameResult &traffic_light_frame_result) {
    traffic_light_frame_result_ = traffic_light_frame_result;
    return true;
}

void PlanningComponent::UpdateHistoryTrajectory(const PosFromIns &pos_ins) {
    if (history_trajectory_.empty() == true) {
        history_trajectory_.push_back(
            Vec2d(pos_ins.position.x, pos_ins.position.y));
    } else {
        double dx = pos_ins.position.x - history_trajectory_.back().x();
        double dy = pos_ins.position.y - history_trajectory_.back().y();
        if (dx * dx + dy * dy > 1.0) {
            history_trajectory_.push_back(
                Vec2d(pos_ins.position.x, pos_ins.position.y));
        }
    }
    if (history_trajectory_.size() >= 31) {
        history_trajectory_.pop_front();
    }
}

}  // namespace planning_lib
}  // namespace jarvis
