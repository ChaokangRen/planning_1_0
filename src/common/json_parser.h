#pragma once

#include <jsoncpp/json/json.h>

#include "common.h"

namespace jarvis {
namespace planning_lib {
class JsonParser {
public:
    JsonParser() = default;

    bool ParserPlanningConf(const std::string &path);

    const PlanningConf GetPlanningConf() {
        return planning_conf_;
    }
    const VehicleParams GetVehicleParams() {
        return planning_conf_.vehicle_params;
    }
    const ReferenceLineSmoothConf GetReferenceLineSmoothConf() {
        return planning_conf_.reference_line_smooth_conf;
    }
    const PathOptimizerConf GetPathOptimizerConf() {
        return planning_conf_.path_optimizer_conf;
    }
    const SpeedOptimizerConf GetSpeedOptimizerConf() {
        return planning_conf_.speed_optimizer_conf;
    }

private:
    PlanningConf planning_conf_;
};
}  // namespace planning_lib
}  // namespace jarvis