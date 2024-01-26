#pragma once

#include <Eigen/Eigen>
#include <list>

#include "common.h"
#include "geodetic_conv.hpp"
#include "planning_interface.h"

namespace jarvis {
namespace planning_lib {

typedef struct {
    std::string instruction;
    std::string orientation;
    std::string road;
    double distance = 0;
    std::vector<Point3d> polyline;
    std::string action;
} PathStep;

const double orgin_longitude1 = 118.87216691555798;
const double orgin_latitude1 = 32.00689537052535;
const double orgin_altitude1 = 13.039715914208223;

class Routing {
public:
    Routing() {
        InitByGaodeMap();
    };

    void Init(const std::vector<RoutingNode> &routing_node);

    bool InitByGaodeMap();

    bool GetNextJunctionMsg(const PosFromIns &vehicle_state,
                            RoutingMsg &routing_msg);

private:
    void Wgs82ToLocal(
        const std::vector<std::pair<double, double>> &wgs_84_points,
        PathStep &path_step);

private:
    std::list<RoutingNode> routing_nodes_;
    std::vector<PathStep> routing_steps_;
    geodetic_converter::GeodeticConverter geodetic_converter_;
    int32_t current_road_cnt = 0;

    double lane_change_dist_sqr = 40000;

    bool first_hit_ = true;
};
}  // namespace planning_lib
}  // namespace jarvis