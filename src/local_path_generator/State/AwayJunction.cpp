#include "AwayJunction.h"
#include "local_path_common.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {
void AwayJunctionState::AwayJunctionCurve(
    const PosFromIns &vehicle_state,
    const std::vector<Point3d> &left_curvature_line, const Injector &injector,
    const RoutingMsg &routing_msg, std::vector<Point3d> &lateral_extended_line,
    std::vector<Point3d> &curvature_line) {
    if (injector.has_stable_slope == true &&
        (routing_msg.turn == TURN::LEFT || routing_msg.turn == TURN::RIGHT)) {
        double dx = lateral_extended_line.front().x - vehicle_state.position.x;
        double dy = lateral_extended_line.front().y - vehicle_state.position.y;
        double rotation = -(M_PI / 2 - vehicle_state.yaw);
        double anchor_x = dx * std::cos(rotation) + dy * std::sin(rotation);
        double anchor_y = -dx * std::sin(rotation) + dy * std::cos(rotation);

        double slope_thata = std::atan(injector.stable_slope);
        double cos_slope_thata = std::cos(slope_thata);
        double sin_slope_thata = std::sin(slope_thata);

        lateral_extended_line.clear();

        for (float s = 0.0; s <= 100; s += 1.0) {
            double x = anchor_x + s * sin_slope_thata;
            double y = anchor_y + s * cos_slope_thata;

            double enu_x = x * injector.cos_trans_theta +
                           y * injector.sin_trans_theta +
                           vehicle_state.position.x;
            double enu_y = -x * injector.sin_trans_theta +
                           y * injector.cos_trans_theta +
                           vehicle_state.position.y;
            lateral_extended_line.emplace_back(Point3d{enu_x, enu_y, 0});
        }
    } else {
    }
    SearchNearestCurve(vehicle_state, left_curvature_line,
                       lateral_extended_line, curvature_line);
}
}  // namespace planning_lib
}  // namespace jarvis