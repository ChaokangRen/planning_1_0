#pragma once
#include <sgtime/sgtime.h>
#include <stdint.h>

#include <string>
#include <vector>

namespace jarvis {
namespace planning_lib {

struct PlanningConfigPath {
    std::string planning_config_path;
    std::string rtk_path;
    std::string sim_map_path;  // For getting ground truth hd-map at simulating.
};

struct Header {
    uint32_t seq;
    jarvis::NanoSecond stamp;
    std::string frame_id;
};

struct Point3d {
    double x = 0;
    double y = 0;
    double z = 0;
};

struct Location {
    Point3d position;  // enu: x y z
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

struct PosFromIns {
    Header header;
    Point3d position;  // enu: x y z
    Point3d velocity;
    Point3d acceleration;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

struct SpeedFromVeh {
    Header header;
    double speed_mps = 0.0;
};

struct WheelSpeedFromVeh {
    Header header;
    double fl_mps = 0.0;
    double fr_mps = 0.0;
    double rl_mps = 0.0;
    double rr_mps = 0.0;
};

struct PathPoint {
    Point3d position_m;

    // direction on the x-y plane
    double theta_rad = 0.0;
    double kappa = 0.0;

    double s_m = 0.0;

    double dkappa = 0.0;
    double ddkappa = 0.0;
};

struct TrajectoryPoint {
    PathPoint path_point;
    double velocity_mps = 0.0;
    double acceleration = 0.0;
    double relative_time_s = 0.0;
    double dot_acc = 0.0;
};

struct TrajectoryMsg {
    std::vector<TrajectoryPoint> trajectory;
    jarvis::NanoSecond timestamp;
};

enum EMERGENCY_LEVEL { NORMAL = 0, WARNING = 1, CRITICAL = 2 };

struct VirtualWall {
    Point3d center_point_m;  // Wall center x-y-z under imu.
    double theta_rad;        // Wall's positive orientation(yaw).
    double width;
    double height;
    double thickness = 0.0;
    EMERGENCY_LEVEL emergency_level = EMERGENCY_LEVEL::NORMAL;
};

typedef std::vector<VirtualWall> VirtualWallsMsg;  // Multiply walls.

enum class ObsType {
    VEHICLE = 0,
    CYCLIST = 1,
    PEDESTRIAN = 2,
    CONE = 3,
    UNKNOWN = 4,
    UNKNOWN_MOVABLE = 5,
    UNKNOWN_UNMOVABLE = 6
};

struct ObsState {
    Point3d position;
    Point3d linear_velocity;
    Point3d linear_acceleration;
    Point3d angular_deg;  // In degree here.
    Point3d angular_velocity;
};

struct ObsTrajectory {
    std::vector<TrajectoryPoint> points;
    double possibility =
        1.0;  // Possibility of this trajectory, from 0.0 to 1.0.
};

typedef std::vector<ObsTrajectory> ObsTrajectories;  // Trajectories.

// enum ObsBehavior {}; // For further implementation..

struct ObstacleMsg {
    ObsType type;
    ObsState state;
    uint32_t track_count;
    std::string track_id;
    double track_time;
    double length;
    double width;
    double height;
    bool is_static;

    // ObsBehavior behavior;          // For further implementation..
    ObsTrajectories
        obs_trajectories;  // Support for empty(no prediction accessed)
                           // and multiply(multiply returns).
};

struct ObstaclesInfo {
    std::vector<ObstacleMsg> obstacles;
    Location location;
    Header header;
};

enum class LaneLineType {
    SOLID = 0,
    BROKEN = 1,
    SOLID_SOLID = 2,
    SOLID_BROKEN = 3,
    BROKEN_SOLID = 4,
    BROKEN_BROKEN = 5,
    SPECIAL = 6
};

enum class LaneLineColor {
    COLOR_WHITE = 1,
    COLOR_YELLOW = 2,
    COLOR_RED = 3,
    COLOR_GREEN = 4,
    COLOR_BLUE = 5,
    COLOR_ORANGE = 6,
    COLOR_VIOLET = 7
};

struct LanePoint {
    PathPoint point;
    LaneLineType type;
    LaneLineColor color_type;
};

struct LaneLineMsg {
    std::vector<LanePoint> lane_line;
    LaneLineType lane_line_type;
    int32_t lane_id;
    enum class Type {
        LANE_LANE = 0,
        SPEED_BUMP,
        STOP_LINE,
        MEDIAN_STRIP,
        CURB
    };
    Type type;
    float confidence;
};

struct LandMark {
    enum class Type { DIRECTION = 0, SPEED_LIMIT, MERGE };

    uint32_t mark_id;
    Type type;
    Point3d center_point;

    bool on_opposite_lane;
    bool can_straight;
    bool can_left;
    bool can_right;
    bool can_turn_around;

    uint32_t minimum_speed_limit;
    uint32_t maximum_speed_limit;

    enum class MergeType { LEFT = 0, RIGHT };
    MergeType merge_type;

    float confidence;
};

struct CenterLine {
    std::vector<PathPoint> points;
    uint32_t id;
};

struct LaneInfo {
    std::vector<LaneLineMsg> lane_lines_msg;
    std::vector<LandMark> land_mark;
    std::vector<CenterLine> center_line;
    Location location;
    Header header;
};

// traffic light data struct
enum TRAFFICLIGHT_COLOR {
    TRAFFICLIGHT_COLOR_RED = 0,
    TRAFFICLIGHT_COLOR_GREEN,
    TRAFFICLIGHT_COLOR_YELLOW,
    TRAFFICLIGHT_COLOR_DARK,
    TRAFFICLIGHT_COLOR_YELLOW_BLINK,
    TRAFFICLIGHT_COLOR_UNKNOWN = -1
};

struct TrafficLightSemantic {
    TRAFFICLIGHT_COLOR color = TRAFFICLIGHT_COLOR_UNKNOWN;
    float distance = 0.0;
    bool is_blink = false;
};

struct TrafficLightDigit {
    TRAFFICLIGHT_COLOR color = TRAFFICLIGHT_COLOR_UNKNOWN;
    std::string digit_num = "NN";
};

struct TrafficLightFrameResult {
    TrafficLightSemantic semantic_straight;
    TrafficLightSemantic semantic_left;
    TrafficLightSemantic semantic_right;
    TrafficLightSemantic semantic_uturn;
    TrafficLightDigit digit;
    // ...... other semantics ......

    Location location;
    Header header;
};

class PlanningInterface {
public:
    virtual ~PlanningInterface(){};

    static PlanningInterface *create_instance();

    virtual std::string get_version() = 0;
    virtual bool init(const PlanningConfigPath &planning_config_path,
                      const std::vector<PosFromIns> &history_ins_pts) = 0;
    virtual bool execute(TrajectoryMsg *trajectory_msg,
                         VirtualWallsMsg *virtual_walls_msg,
                         std::string &debuginfo) = 0;

    virtual bool set_pos_from_ins(const PosFromIns &pos_ins) = 0;
    virtual bool set_speed_from_veh(const SpeedFromVeh &speed_veh) = 0;
    virtual bool set_wheel_speed_from_veh(
        const WheelSpeedFromVeh &wheel_speed_veh) = 0;
    virtual bool set_obstacles_info(const ObstaclesInfo &obstacle_info) = 0;
    virtual bool set_lane_info(const LaneInfo &lane_info) = 0;
    virtual bool set_traffic_light_frame_result(
        const TrafficLightFrameResult &traffic_light_frame_result) = 0;
};

}  // namespace planning_lib
}  // namespace jarvis