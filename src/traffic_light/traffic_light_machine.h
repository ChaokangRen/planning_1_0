#pragma once

#include "common.h"
#include "planning_interface.h"

namespace jarvis {
namespace planning_lib {

class TrafficLightMachine {
public:
    bool Process(const TrafficLightSemantic &traffic_light_semantic,
                 const TrafficLightDigit &traffic_digit);

    int32_t TrafficDigitNum(void) const {
        return traffic_digit_num_;
    }

    TRAFFICLIGHT_COLOR TrafficColor(void) const {
        return traffic_color_;
    }

    TRAFFICLIGHT_COLOR DigitColor(void) const {
        return digit_color;
    }

    bool IsBlink(void) const {
        return is_blink_;
    }

    bool HasDigitNum() const {
        return has_digit_num_;
    }

    bool CanGo(void) {
        return traffic_color_ == TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN;
    }
    void Reset(void) {
        traffic_color_ = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN;
        digit_color = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN;
        traffic_digit_num_ = -1;
        has_digit_num_ = false;
        is_blink_ = false;
    }
    void SetDigitNum(const int32_t cnt) {
        traffic_digit_num_ = cnt;
    }

    void SetTrafficColor(const TRAFFICLIGHT_COLOR color) {
        traffic_color_ = color;
    }

private:
    TRAFFICLIGHT_COLOR traffic_color_ =
        TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN;

    int32_t traffic_digit_num_ = -1;

    TRAFFICLIGHT_COLOR digit_color =
        TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN;

    int32_t cnt_ = 0;

    bool has_digit_num_ = false;

    bool is_blink_ = false;
};
}  // namespace planning_lib
}  // namespace jarvis