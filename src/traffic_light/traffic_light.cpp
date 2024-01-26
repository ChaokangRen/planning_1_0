#include "traffic_light.h"

#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {

void TrafficLight::Init(const TrafficLightConf &traffic_light_conf) {
    Json::Reader reader;
    Json::Value root;
    std::string traffic_light_path =
        "/usr/share/jarvis/planning/resource/rtk_trajectory/"
        "traffic_light.json";
    std::ifstream in(traffic_light_path, std::ios::binary);

    if (!in.is_open()) {
        SG_ERROR("json file open failed!");
    }
    if (reader.parse(in, root)) {
        int32_t traffic_size = root["traffic_light_point"].size();
        const auto &root_ref = root["traffic_light_point"];
        for (int32_t i = 0; i < traffic_size; ++i) {
            const auto &point = root_ref[i];
            TrafficLightMsg traffic_msg;
            traffic_msg.x = point[0].asDouble();
            traffic_msg.y = point[1].asDouble();
            traffic_msg.yaw = point[2].asDouble();
            traffic_msg.which_light = GetWhichLight(point[3].asInt());
            traffic_light_list_.emplace_back(traffic_msg);
        }
    }

    virtual_wall_length_ = traffic_light_conf.virtual_wall_length;
    virtual_wall_width_ = traffic_light_conf.virtual_wall_width;
    virtual_wall_height_ = traffic_light_conf.virtual_wall_height;

    TrafficLightReset();
}

WHICHLIGHT TrafficLight::GetWhichLight(int32_t count) {
    if (count == 0) {
        return WHICHLIGHT::LEFT;
    }
    if (count == 1) {
        return WHICHLIGHT::MIDDLE;
    }
    if (count == 2) {
        return WHICHLIGHT::RIGHT;
    }
    return WHICHLIGHT::NONE;
}

void TrafficLight::Process(
    const TrafficLightFrameResult &traffic_light_frame_result) {
    traffic_light_frame_result_ = traffic_light_frame_result;

    // // determine traffic_light existence
    is_enable_ = (traffic_light_frame_result_.semantic_left.color !=
                      TRAFFICLIGHT_COLOR_UNKNOWN ||
                  traffic_light_frame_result_.semantic_straight.color !=
                      TRAFFICLIGHT_COLOR_UNKNOWN ||
                  traffic_light_frame_result_.semantic_right.color !=
                      TRAFFICLIGHT_COLOR_UNKNOWN ||
                  traffic_light_frame_result_.semantic_uturn.color !=
                      TRAFFICLIGHT_COLOR_UNKNOWN);

    left_machine_.Process(traffic_light_frame_result_.semantic_left,
                          traffic_light_frame_result_.digit);

    right_machine_.Process(traffic_light_frame_result_.semantic_right,
                           traffic_light_frame_result_.digit);

    straight_machine_.Process(traffic_light_frame_result_.semantic_straight,
                              traffic_light_frame_result_.digit);

    around_machine_.Process(traffic_light_frame_result_.semantic_uturn,
                            traffic_light_frame_result_.digit);
    // SG_WARN("left color = %d,digit = %d,straight color = %d,digit = %d",
    //         left_machine_.TrafficColor(), left_machine_.TrafficDigitNum(),
    //         straight_machine_.TrafficColor(),
    //         straight_machine_.TrafficDigitNum());
    if (left_machine_.TrafficColor() ==
        TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED) {
        if (straight_machine_.TrafficColor() ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED) {
            left_machine_.SetDigitNum(straight_machine_.TrafficDigitNum() + 25);
            // SG_INFO("1");
        } else if (straight_machine_.TrafficColor() ==
                   TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN) {
            left_machine_.SetDigitNum(straight_machine_.TrafficDigitNum());
        }
    }

    if (left_machine_.TrafficColor() ==
        TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN) {
        // SG_INFO("3left_machine_ = %d", left_machine_.TrafficColor());
        straight_machine_.SetDigitNum(left_machine_.TrafficDigitNum() + 25);
    }

    // SG_INFO("left color = %d,digit = %d,straight color = %d,digit = %d",
    //         left_machine_.TrafficColor(), left_machine_.TrafficDigitNum(),
    //         straight_machine_.TrafficColor(),
    //         straight_machine_.TrafficDigitNum());

    if (is_enable_ == false) {
        if (left_machine_.DigitColor() ==
                TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED ||
            right_machine_.DigitColor() ==
                TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED ||
            straight_machine_.DigitColor() ==
                TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED) {
            is_enable_ = true;
        }
    }

    if (is_enable_ == false) {
        left_machine_.Reset();
        right_machine_.Reset();
        straight_machine_.Reset();
        around_machine_.Reset();
        green_status_ = 0;
    }
}

VirtualObs TrafficLight::GenerateVirtualWall(const PosFromIns &pos_ins,
                                             const double &distance) {
    double veh_x = pos_ins.position.x;
    double veh_y = pos_ins.position.y;
    double veh_yaw = pos_ins.yaw;
    double wall_x = veh_x + distance * std::cos(veh_yaw);
    double wall_y = veh_y + distance * std::sin(veh_yaw);
    double wall_yaw = veh_yaw;
    return VirtualObs(wall_x, wall_y, wall_yaw, virtual_wall_length_,
                      virtual_wall_width_, virtual_wall_height_, pos_ins);
}

void TrafficLight::VirtualWallProcess(const Lane &lane,
                                      const PosFromIns &pos_ins,
                                      const ReferenceLine &reference_line,
                                      VirtualWallsMsg *virtual_walls_msg,
                                      std::vector<Obstacle> &obstacles) {
    if (!is_enable_) {
        return;
    }
    WHICHLIGHT intension = GetIntension(pos_ins);

    double vel = std::sqrt(pos_ins.velocity.x * pos_ins.velocity.x +
                           pos_ins.velocity.y * pos_ins.velocity.y);
    double ref_vel = reference_line.RefSpeed();
    double veh2stopline_dist = lane.GetDistance2StopLine();
    if (lane.Status() == StopLineStatus::None ||
        intension == WHICHLIGHT::NONE) {
        Reset();
        return;
    }
    if (veh2stopline_dist > virtual_wall_active_distance_) {
        return;
    }

    double average_vel = AverageSpeed(vel, ref_vel, 1.0, veh2stopline_dist);

    TrafficLightMachine &traffic_machine = left_machine_;
    if (intension == WHICHLIGHT::LEFT) {
        traffic_machine = left_machine_;
        // SG_INFO("left");
    } else if (intension == WHICHLIGHT::RIGHT) {
        traffic_machine = right_machine_;
        // SG_INFO("right");
    } else if (intension == WHICHLIGHT::MIDDLE) {
        traffic_machine = straight_machine_;
        // SG_INFO("straight");
    } else if (intension == WHICHLIGHT::TURNAROUND) {
        traffic_machine = around_machine_;
        // SG_INFO("around");
    }

    is_generator_wall_ =
        TrafficeLightProcess(veh2stopline_dist, vel, traffic_machine);

    if (veh2stopline_dist > 1.0) {
        veh2stopline_dist -= 1.0;
    } else {
        veh2stopline_dist = 0.0;
    }

    VirtualObs virtual_obstacle =
        GenerateVirtualWall(pos_ins, veh2stopline_dist);
    if (is_generator_wall_ == true) {
        obstacles.emplace_back(virtual_obstacle.GetObstacleMsg());
        obstacles.back().SetSlBoundarys(reference_line.GetSlBoundary(
            obstacles.back().GetObstaclePolygon()));
        virtual_walls_msg->emplace_back(virtual_obstacle.GetVirtualWall());
        // SG_INFO("has virtual wall");
    } else {
        // SG_INFO("no virtual wall");
    }
}

bool TrafficLight::TrafficeLightProcess(
    const double distance, const double vel,
    const TrafficLightMachine &traffic_machine) {
    double veh2stopline_time = distance / vel;
    // SG_INFO("vel = %lf,dist = %lf,time = %lf", vel, distance,
    //         veh2stopline_time);
    if (traffic_machine.TrafficColor() ==
        TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED) {
        is_lock_green_light_wall_ = false;
        is_lock_green_light_wall_pass_ = false;
        if (is_lock_red_light_wall_ == true) {
            return true;
        }
        if (is_lock_red_light_wall_pass_ == true) {
            return false;
        }
        if (vel < stop_vel_red_) {
            return true;
        } else {
            if (traffic_machine.HasDigitNum() == true) {
                // SG_INFO("traffic_machine.TrafficDigitNum() = %d",
                //         traffic_machine.TrafficDigitNum() + 2);
                if (veh2stopline_time < traffic_machine.TrafficDigitNum() + 1) {
                    is_lock_red_light_wall_ = true;
                    return true;
                } else {
                    if (traffic_machine.TrafficDigitNum() + 1 < 5) {
                        is_lock_red_light_wall_pass_ = true;
                    }
                    return false;
                }
            } else {
                if (traffic_machine.IsBlink() == true) {
                    return true;
                } else {
                    return true;
                }
            }
        }
        green_status_ = 0;

    } else if (traffic_machine.TrafficColor() ==
               TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN) {
        is_lock_red_light_wall_ = false;
        is_lock_red_light_wall_pass_ = false;
        if (is_lock_green_light_wall_ == true) {
            return true;
        }
        if (is_lock_green_light_wall_pass_ == true) {
            return false;
        }
        if (vel > stop_vel_green_) {
            if (traffic_machine.HasDigitNum() == true) {
                if (veh2stopline_time > traffic_machine.TrafficDigitNum()) {
                    is_lock_green_light_wall_ = true;
                    green_status_ = 1;
                    return true;
                } else {
                    if (traffic_machine.TrafficDigitNum() < 5) {
                        is_lock_green_light_wall_pass_ = true;
                    }
                    green_status_ = 2;
                    return false;
                }
            } else {
                if (traffic_machine.IsBlink() == true) {
                    if (veh2stopline_time > blink_time_lo_) {
                        green_status_ = 1;
                        return true;
                    } else {
                        green_status_ = 2;
                        return false;
                    }
                } else {
                    green_status_ = 2;
                    return false;
                }
            }
        } else {
            green_status_ = 2;
            return false;
        }

    } else if (traffic_machine.TrafficColor() ==
               TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_YELLOW) {
        if (green_status_ == 0) {
            if (distance > -1.0) {
                return true;
            } else {
                return false;
            }
        } else if (green_status_ == 1) {
            return true;
        } else if (green_status_ == 2) {
            return false;
        }
    } else {
        is_lock_red_light_wall_ = false;
        is_lock_green_light_wall_ = false;
        is_lock_green_light_wall_pass_ = false;
        is_lock_red_light_wall_pass_ = false;
    }
    return false;
}

bool TrafficLight::IsEnable() {
    return is_enable_;
}

void TrafficLight::TrafficLightReset() {
    traffic_light_frame_result_.semantic_left.color =
        TRAFFICLIGHT_COLOR_UNKNOWN;
    traffic_light_frame_result_.semantic_straight.color =
        TRAFFICLIGHT_COLOR_UNKNOWN;
    traffic_light_frame_result_.semantic_right.color =
        TRAFFICLIGHT_COLOR_UNKNOWN;
    traffic_light_frame_result_.semantic_uturn.color =
        TRAFFICLIGHT_COLOR_UNKNOWN;
    traffic_light_frame_result_.digit.digit_num = "NN";
}

WHICHLIGHT TrafficLight::GetIntension(const PosFromIns &pos_ins) {
    double veh_x = pos_ins.position.x;
    double veh_y = pos_ins.position.y;

    double min_distance = 1e10;
    int32_t min_count = traffic_light_list_.size();
    WHICHLIGHT light = WHICHLIGHT::NONE;

    for (int32_t i = 0; i < traffic_light_list_.size(); ++i) {
        double dx = veh_x - traffic_light_list_[i].x;
        double dy = veh_y - traffic_light_list_[i].y;

        double dist = std::sqrt(dx * dx + dy * dy);
        double yaw_error = std::fabs(traffic_light_list_[i].yaw - pos_ins.yaw);
        if (dist < min_distance && yaw_error < 0.785) {
            min_distance = dist;
            min_count = i;
            light = traffic_light_list_[i].which_light;
        }
    }
    double dx = traffic_light_list_[min_count].x - veh_x;
    double dy = traffic_light_list_[min_count].y - veh_y;
    double sin_yaw = std::sin(pos_ins.yaw);
    double cos_yaw = std::cos(pos_ins.yaw);
    double ex = dx * cos_yaw + dy * sin_yaw;
    double ey = -dx * sin_yaw + dy * cos_yaw;

    if (ex < -10) {
        light = WHICHLIGHT::NONE;
        // SG_INFO("light none");
    }

    if (min_distance > 80) {
        light = WHICHLIGHT::NONE;
    }

    return light;
}
void TrafficLight::Reset(void) {
    green_status_ = 0;
    is_lock_red_light_wall_ = false;
    is_lock_red_light_wall_pass_ = false;
    is_lock_green_light_wall_ = false;
    is_lock_green_light_wall_pass_ = false;
    left_machine_.Reset();
    right_machine_.Reset();
    straight_machine_.Reset();
    around_machine_.Reset();
}

}  // namespace planning_lib
}  // namespace jarvis