
#include "sglog/sglog.h"
#include "traffic_light_machine.h"

namespace jarvis {
namespace planning_lib {

bool TrafficLightMachine::Process(
    const TrafficLightSemantic &traffic_light_semantic,
    const TrafficLightDigit &traffic_digit) {
    is_blink_ = traffic_light_semantic.is_blink;
    if (traffic_light_semantic.color ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN ||
        traffic_light_semantic.color ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED ||
        traffic_light_semantic.color ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_YELLOW) {
        traffic_color_ = traffic_light_semantic.color;
        if (traffic_color_ == traffic_digit.color &&
            IsNumeric(traffic_digit.digit_num) == true) {
            traffic_digit_num_ = std::stod(traffic_digit.digit_num);
            has_digit_num_ = true;
        } else {
            traffic_digit_num_ = -1;
            has_digit_num_ = false;
        }
        digit_color = traffic_digit.color;
        cnt_ = 0;
    } else {
        ++cnt_;
        if (traffic_digit_num_ >= 0 && traffic_digit_num_ <= 99) {
            if (cnt_ >= 10) {
                cnt_ = 0;
                --traffic_digit_num_;
            }
        }
        if (traffic_digit_num_ < 0) {
            if (traffic_color_ ==
                TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN) {
                traffic_color_ = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_YELLOW;
                traffic_digit_num_ = 3;
                digit_color = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_YELLOW;
            }
            if (traffic_color_ ==
                TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_YELLOW) {
                traffic_color_ = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED;
                traffic_digit_num_ = 30;
                digit_color = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED;
            }
            if (traffic_color_ == TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_RED) {
                traffic_color_ = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN;
                traffic_digit_num_ = 30;
                digit_color = TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_GREEN;
            }
        }
    }
    return true;
}

}  // namespace planning_lib
}  // namespace jarvis