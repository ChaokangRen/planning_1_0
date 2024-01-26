#include "planning_interface.h"

namespace jarvis {
namespace planning_lib {

class ObstacleGenerator {
public:
    ObstacleGenerator() = default;
    void InitObstacleTrajectory();
    std::vector<PathPoint> Obstacle1Path() {
        return obstacle1_path;
    }

    bool UpdateObstacle(const PosFromIns &pos_ins, ObstacleMsg *obstacle);

private:
    std::vector<PathPoint> obstacle1_path;
};
}  // namespace planning_lib
}  // namespace jarvis