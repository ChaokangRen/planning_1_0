#pragma once

#include "box2d.h"
#include "common.h"
#include "polygon2d.h"
#include "sglog/sglog.h"
#include "st_boundary.h"

namespace jarvis {
namespace planning_lib {

class Obstacle {
public:
    enum class BEHAVIORTYPE {
        IGNORE = 0,
        OVERTAKE = 1,
        FOLLOW = 2,
        YIELD = 3,
        STOP = 4,
        LEFTNUDGE = 5,
        RIGHTNUDGE = 6,
        NORMAL = 7,
        UNCERTAIN = 8
    };

    enum class POSITIONTYPE {
        NONE = 0,
        FRONT = 1,
        FRONTLEFT = 2,
        FRONTRIGHT = 3,
        REARLEFT = 4,
        REARRIGHT = 5
    };

public:
    Obstacle() = default;

    Obstacle(const ObstacleMsg &obstacle_msg) : obstacle_msg_(obstacle_msg) {
        obstacle_box_ = Box2d(Vec2d(obstacle_msg_.state.position.x,
                                    obstacle_msg_.state.position.y),
                              obstacle_msg_.state.angular_deg.z,
                              obstacle_msg_.length, obstacle_msg_.width);
        obstacle_polygon_ = Polygon2d(obstacle_box_);
        speed_ = std::sqrt(obstacle_msg_.state.linear_velocity.x *
                               obstacle_msg_.state.linear_velocity.x +
                           obstacle_msg_.state.linear_velocity.y *
                               obstacle_msg_.state.linear_velocity.y);
        // if (!obstacle_msg.obs_trajectories.empty()) {
        //     obstacle_msg_.is_static = false;
        //     is_static_ = false;
        // } else {
        //     obstacle_msg_.is_static = true;
        //     is_static_ = true;
        // }
        if (speed_ < static_speed_) {
            obstacle_msg_.is_static = true;
            is_static_ = true;
        } else {
            obstacle_msg_.is_static = false;
            is_static_ = false;
        }
    }

    bool IsStatic() const {
        return is_static_;
    }

    void SetStatic(const bool is_static) {
        is_static_ = is_static;
    }

    bool IsVirtual() const {
        return is_virtual_;
    }

    void SetVirtual(bool is_virtual) {
        is_virtual_ = is_virtual;
    }

    void SetBehavior(const BEHAVIORTYPE behavior) {
        behaviour_ = behavior;
    }

    BEHAVIORTYPE Behavior(void) const {
        return behaviour_;
    }

    void SetPositionType(const POSITIONTYPE position_type) {
        position_type_ = position_type;
    }

    POSITIONTYPE PositionType(void) const {
        return position_type_;
    }

    void SetStBoundary(const StBoundary &st_boundary) {
        st_boundary_ = st_boundary;
    }

    StBoundary GetStBoundary(void) {
        return st_boundary_;
    }

    double Speed() const;

    const std::string &Id() const;

    const Polygon2d &GetObstaclePolygon() const;

    const Box2d &GetObstacleBox() const;

    const ObsTrajectory &Trajectorys() const;

    Box2d GetBoundingBox(const PathPoint &point) const;

    bool GetPointAtTime(const double t, PathPoint *path_point) const;

    bool HasTrajectory() const {
        return !(obstacle_msg_.obs_trajectories.empty());
    }

    void SetSlBoundarys(const SLStaticBoundary &obs_sl_boundary) {
        obs_sl_boundary_ = obs_sl_boundary;
    }

    SLStaticBoundary GetObsSlBoundary() const {
        return obs_sl_boundary_;
    }
    void SetPosInVeh(const Vec2d &pos) {
        pos_in_veh_ = pos;
    }
    Vec2d GetPosInVeh(void) {
        return pos_in_veh_;
    }
    double Heading() const {
        return obstacle_msg_.state.angular_deg.z;
    }
    double Length() const {
        return obstacle_msg_.length;
    }
    double Width() const {
        return obstacle_msg_.width;
    }

    Vec2d Center() const {
        return obstacle_box_.Center();
    }

private:
    std::string id_;

    int32_t perception_id_ = 0;

    bool is_static_ = false;

    bool is_virtual_ = false;

    ObstacleMsg obstacle_msg_;

    double speed_ = 0.0;
    double reponse_time_ = 1.0;

    Polygon2d obstacle_polygon_;
    Box2d obstacle_box_;

    SLStaticBoundary obs_sl_boundary_;

    StBoundary st_boundary_;

    BEHAVIORTYPE behaviour_ = BEHAVIORTYPE::NORMAL;
    POSITIONTYPE position_type_ = POSITIONTYPE::NONE;
    double static_speed_ = 0.1;

    Vec2d pos_in_veh_;
};
}  // namespace planning_lib
}  // namespace jarvis