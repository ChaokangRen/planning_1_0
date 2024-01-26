#include "OnJunction.h"
#include "local_path_common.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {

bool OnJunctionState::Process(const LaneInfo &lane_info,
                              const PosFromIns &vehicle_state,
                              const RoutingMsg &routing_msg,
                              std::vector<Point3d> &curvature_line) {
    //   SearchNearestCurve(vehicle_state, curvature_line);
    return false;
}
void OnJunctionState::OnJunctionCurve(
    const PosFromIns &vehicle_state,
    const std::vector<Point3d> &left_curvature_line,
    const std::vector<Point3d> &lateral_extended_line,
    std::vector<Point3d> &curvature_line) {
    SearchNearestCurve(vehicle_state, left_curvature_line,
                       lateral_extended_line, curvature_line);
}
}  // namespace planning_lib
}  // namespace jarvis