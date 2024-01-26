#pragma once
#include "common.h"
#include "planning_interface.h"
namespace jarvis {
namespace planning_lib {
class OnJunctionState {
public:
    bool Process(const LaneInfo &lane_info, const PosFromIns &vehicle_state,
                 const RoutingMsg &routing_msg,
                 std::vector<Point3d> &curvature_line);
    void OnJunctionCurve(const PosFromIns &vehicle_state,
                         const std::vector<Point3d> &left_curvature_line,
                         const std::vector<Point3d> &lateral_extended_line,
                         std::vector<Point3d> &curvature_line);

private:
    std::vector<Point3d> junction_line_;
    std::vector<Point3d> lateral_extended_line_;
};
}  // namespace planning_lib
}  // namespace jarvis