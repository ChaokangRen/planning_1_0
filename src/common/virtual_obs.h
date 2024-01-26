#pragma once

#include "common.h"
#include "obstacle.h"
#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {
class VirtualObs {
public:
    VirtualObs() = default;
    VirtualObs(const VirtualObs &virtual_obs) {
        virtual_wall_ = virtual_obs.virtual_wall_;
        total_time_ = virtual_obs.total_time_;
        obstacle_ = virtual_obs.obstacle_;
        virtual_length_ = virtual_obs.virtual_length_;
        virtual_width_ = virtual_obs.virtual_width_;
        virtual_height_ = virtual_obs.virtual_height_;
        obstacle_.SetVirtual(true);
    }
    VirtualObs(const double x, const double y, const double yaw,
               const double length, const double width, const double height,
               const PosFromIns &pos_ins);
    Obstacle GetObstacleMsg() {
        return obstacle_;
    }
    VirtualWall GetVirtualWall() {
        return virtual_wall_;
    }
    void SetObstacle(const Obstacle &obs) {
        obstacle_ = obs;
    }
    void SetVirtualWall(const VirtualWall &virtual_wall) {
        virtual_wall_ = virtual_wall;
    }
    bool IsActived() {
        return is_actived_;
    }
    void SetActived(const bool is_actived) {
        is_actived_ = is_actived;
    }

private:
    VirtualWall virtual_wall_;
    Obstacle obstacle_;

    double total_time_ = 8.0;
    double virtual_length_ = 0.0;
    double virtual_width_ = 0.0;
    double virtual_height_ = 0.0;
    bool is_actived_ = false;
};
}  // namespace planning_lib
}  // namespace jarvis