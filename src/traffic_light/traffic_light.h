#pragma once
#include <jsoncpp/json/json.h>

#include <fstream>

#include "common.h"
#include "lane.h"
#include "planning_interface.h"
#include "reference_line.h"
#include "traffic_light_machine.h"
#include "virtual_obs.h"

namespace jarvis {
namespace planning_lib {

enum class WHICHLIGHT {
    LEFT = 0,
    MIDDLE = 1,
    RIGHT = 2,
    TURNAROUND = 3,
    NONE = 4
};

struct TrafficLightMsg {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    WHICHLIGHT which_light;
};

class TrafficLight {
public:
    void Init(const TrafficLightConf &traffic_light_conf);

    void Process(const TrafficLightFrameResult &traffic_light_frame_result);

    void VirtualWallProcess(const Lane &lane, const PosFromIns &pos_ins,
                            const ReferenceLine &reference_line,
                            VirtualWallsMsg *virtual_walls_msg,
                            std::vector<Obstacle> &obstacles);

    VirtualObs GenerateVirtualWall(const PosFromIns &pos_ins,
                                   const double &distance);

    bool IsEnable();

    WHICHLIGHT GetIntension(const PosFromIns &pos_ins);

    void Reset(void);

private:
    WHICHLIGHT GetWhichLight(int32_t count);

    void TrafficLightReset();

    bool TrafficeLightProcess(const double distance, const double vel,
                              const TrafficLightMachine &traffic_machine);

    std::vector<TrafficLightMsg> traffic_light_list_;

    TrafficLightFrameResult traffic_light_frame_result_;

    double virtual_wall_length_ = 0.0;

    double virtual_wall_width_ = 0.0;

    double virtual_wall_height_ = 0.0;

    bool is_enable_ = false;

    TrafficLightMachine left_machine_;

    TrafficLightMachine right_machine_;

    TrafficLightMachine straight_machine_;

    TrafficLightMachine around_machine_;

    double virtual_wall_active_distance_ = 60;

    int16_t blink_time_up_ = 3;
    int16_t blink_time_lo_ = 1;
    float stop_vel_red_ = 2.0;
    float stop_vel_green_ = 3.0;

    bool is_generator_wall_ = false;

    int16_t green_status_ = 0;

    bool is_lock_red_light_wall_ = false;
    bool is_lock_red_light_wall_pass_ = false;
    bool is_lock_green_light_wall_pass_ = false;
    bool is_lock_green_light_wall_ = false;
};
}  // namespace planning_lib
}  // namespace jarvis