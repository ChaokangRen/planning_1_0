#pragma once

#include <vector>

#include "planning_interface.h"
#include "sglog/sglog.h"
#include "virtual_obs.h"

namespace jarvis {
namespace planning_lib {
class RtkGenerator {
public:
    RtkGenerator() = default;

    void Init(const std::string &rtk_path, const double refline_total_length) {
        InitRtkTrajectory(rtk_path);
        InitRtkLeftTrajectory(rtk_path);
        local_path_ = std::vector<std::vector<PathPoint>>(
            center_lines_.size(), std::vector<PathPoint>());
        count_vec_ = std::vector<int32_t>(2, 0);
        end_point_.x = 4203.38;
        end_point_.y = 2534.15;
        end_point_.z = 0.8126;
        refline_total_length_ = refline_total_length;
        local_points_num_ = static_cast<int>(refline_total_length_ / delta_s_);
    }
    VirtualObs EndVirtualObs(const PosFromIns &pos_ins);

    bool UpdateLocalPathByRtkTrajectory(PosFromIns pos_ins);
    bool UpdateLocalLeftPathByRtkTrajectory(const PosFromIns &pos_ins);
    bool UpdateLocalPathForPlanning(const PosFromIns &pos_ins,
                                    const int32_t lane_id);
    bool UpdateLocalPathForPlanning(const PosFromIns &pos_ins);

    std::vector<PathPoint> LocalPath(const int32_t lane_id) {
        if (lane_id < 0 || lane_id >= local_path_.size()) {
            SG_ERROR("has no this lane id");
            return local_path_[0];
        }
        return local_path_[lane_id];
    }

    TrajectoryMsg UpdateTrajectory(double x, double y);

    const std::vector<PathPoint> &GlobalPath() {
        return center_lines_[0];
    }

    TrajectoryMsg UpdateTrajectory(double x, double y, double vel);

    bool UpdateObstacle(const PosFromIns &pos_ins, ObstacleMsg *obstacle);

    std::vector<std::vector<PathPoint>> LocalCenterLine() const {
        SG_INFO("local_path_size=%d", local_path_.size());
        return local_path_;
    }

    void GetLaneBoundary(const int32_t land_id, double *left_boundary,
                         double *right_boundary);

    void WriteObsLine();
    ~RtkGenerator() {}

private:
    bool InitRtkTrajectory(const std::string &rtk_path);
    bool InitRtkLeftTrajectory(const std::string &rtk_path);

private:
    std::vector<std::vector<PathPoint>> local_path_;

    int32_t local_points_num_ = 0;

    double delta_s_ = 1.0;

    std::vector<std::vector<PathPoint>> center_lines_;

    int32_t count_ = 0;
    int32_t left_count_ = 0;

    int32_t count_stop_ = 0;

    std::vector<int32_t> count_vec_;

    double vel_final_ = 5;
    double time_gap_ = 0.01;
    double velocity_ = 5;

    Point3d end_point_;

    double refline_total_length_ = 0.0;
};

}  // namespace planning_lib
}  // namespace jarvis