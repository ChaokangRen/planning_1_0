#include <jsoncpp/json/json.h>

#include <fstream>
#include <list>
#include <string>

#include "common.h"
#include "planning_interface.h"
#include "routing.h"
#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

int32_t GetRouteCnt(const std::vector<PathStep> &path_step, const double x,
                    const double y) {
    double dx = 0.0;
    double dy = 0.0;
    double min_dist = 1e10;
    int32_t min_cnt = 0;

    for (int32_t i = 0; i < path_step.size() - 1; ++i) {
        double min_j_dist_sqr = 1e10;
        for (int32_t j = 0; j < path_step[i].polyline.size(); ++j) {
            dx = path_step[i].polyline[j].x - x;
            dy = path_step[i].polyline[j].y - y;
            double dist_sqr = dx * dx + dy * dy;
            if (min_j_dist_sqr > dist_sqr) {
                min_j_dist_sqr = dist_sqr;
            }
        }
        if (min_dist > min_j_dist_sqr) {
            min_dist = min_j_dist_sqr;
            min_cnt = i;
        }
    }

    return min_cnt;
}
// std::vector<std::string> action_set{
//     "turn_left",         "turn_right",        "drive_ahead_left",
//     "drive_ahead_right", "drive_rear_left",   "drive_rear_right",
//     "left_u_turn",       "go_straight",       "keep_to_left",
//     "keep_to_right",     "access_roundabout", "exit_roundabout",
//     "slow_down","none"};
TURN ActionTrans(const std::string action) {
    if (action == "turn_left") {
        return TURN::LEFT;
    }
    if (action == "turn_right") {
        return TURN::RIGHT;
    }
    if (action == "go_straight") {
        return TURN::STRAIGHT;
    }
    if (action == "none") {
        return TURN::NONE;
    }
    return TURN::NONE;
}

void StringSplit(const std::string &polyline,
                 std::vector<std::pair<double, double>> &wgs_84_points) {
    if (polyline.find(";") == std::string::npos) {
        std::string::size_type m = polyline.find(",");
        std::string lon_str = polyline.substr(0, m);
        std::string lat_str = polyline.substr(m + 1);
        wgs_84_points.emplace_back(
            std::make_pair(std::stod(lon_str), std::stod(lat_str)));
        return;
    } else {
        std::string::size_type n = polyline.find(";");
        std::string str1 = polyline.substr(0, n);
        std::string str2 = polyline.substr(n + 1);

        std::string::size_type m = polyline.find(",");
        std::string lon_str = str1.substr(0, m);
        std::string lat_str = str1.substr(m + 1);
        wgs_84_points.emplace_back(
            std::make_pair(std::stod(lon_str), std::stod(lat_str)));
        StringSplit(str2, wgs_84_points);
    }
}

void Routing::Wgs82ToLocal(
    const std::vector<std::pair<double, double>> &wgs_84_points,
    PathStep &path_step) {
    for (const std::pair<double, double> &wgs_point : wgs_84_points) {
        Point3d point{0, 0, 0};
        // geodetic_converter_.geodetic2Enu(wgs_point.second, wgs_point.first,
        //                                  orgin_altitude1, &point.x, &point.y,
        //                                  &point.z);

        geodetic_converter_.gcj02_to_wgs84(wgs_point.second, wgs_point.first,
                                           orgin_altitude1, &point.x, &point.y,
                                           &point.z);
        // point.x -= 471.5;
        // point.y += 244.5;
        // SG_INFO("lon = %lf,lat = %lf,x = %lf,y = %lf", wgs_point.first,
        //         wgs_point.second, point.x, point.y);
        path_step.polyline.emplace_back(point);
    }
}

void Routing::Init(const std::vector<RoutingNode> &routing_node) {
    for (const RoutingNode &node : routing_node) {
        routing_nodes_.emplace_back(node);
    }
    InitByGaodeMap();
}

bool Routing::InitByGaodeMap() {
    Json::Reader reader;
    Json::Value root;
    geodetic_converter_.initialiseReference(orgin_latitude1, orgin_longitude1,
                                            orgin_altitude1);

    std::string gaode_route_path =
        "/usr/share/jarvis/planning/resource/planning_conf/qidi_demo_en.json";

    std::ifstream in(gaode_route_path, std::ios::binary);

    if (!in.is_open()) {
        SG_ERROR("json file open failed!");
    }
    int32_t status = 0;
    std::string status_info;
    double route_distance = 0;

    if (reader.parse(in, root)) {
        status = std::stoi(root["status"].asString());
        status_info = root["info"].asString();
        if (status != 1) {
            SG_ERROR("gaode route get failed! msg = %s", status_info.c_str());
        }
        int32_t path_count = std::stoi(root["count"].asString());
        int32_t default_count = 0;
        const auto &paths = root["route"]["paths"][default_count];
        route_distance = std::stod(paths["distance"].asString().c_str());

        for (const auto &step : paths["steps"]) {
            PathStep path_step;
            path_step.instruction = step["instruction"].asString();
            path_step.orientation = step["orientation"].asString();
            path_step.road = step["road"].asString();
            path_step.distance = std::stod(step["distance"].asString());
            if (step["action"].empty()) {
                path_step.action = "none";
            } else {
                path_step.action = step["action"].asString();
            }

            std::string polyline = step["polyline"].asString();
            std::vector<std::pair<double, double>> wgs_84_points;
            StringSplit(polyline, wgs_84_points);
            Wgs82ToLocal(wgs_84_points, path_step);
            routing_steps_.emplace_back(path_step);
        }
    }
    return true;
}
bool Routing::GetNextJunctionMsg(const PosFromIns &vehicle_state,
                                 RoutingMsg &routing_msg) {
    if (first_hit_ == true) {
        first_hit_ = false;
        current_road_cnt = GetRouteCnt(routing_steps_, vehicle_state.position.x,
                                       vehicle_state.position.y);
    }
    Point3d road_section_point =
        routing_steps_[current_road_cnt].polyline.back();
    double dx = road_section_point.x - vehicle_state.position.x;
    double dy = road_section_point.y - vehicle_state.position.y;
    double cos_car_yaw = std::cos(vehicle_state.yaw);
    double sin_car_yaw = std::sin(vehicle_state.yaw);
    double abs_distance_sqr = dx * dx + dy * dy;

    double longitudinal_s = dx * cos_car_yaw + dy * sin_car_yaw;

    if (longitudinal_s < 0) {
        ++current_road_cnt;
        road_section_point = routing_steps_[current_road_cnt].polyline.back();
        dx = road_section_point.x - vehicle_state.position.x;
        dy = road_section_point.y - vehicle_state.position.y;
        abs_distance_sqr = dx * dx + dy * dy;
        longitudinal_s = dx * cos_car_yaw + dy * sin_car_yaw;
    }
    if (current_road_cnt >= routing_steps_.size()) {
        current_road_cnt = 0;
        // current_road_cnt = GetRouteCnt(routing_steps_,
        // vehicle_state.position.x,
        //                                vehicle_state.position.y);
        // SG_ERROR("has no path,please takeover!");
        // return false;
    }
    // current_road_cnt = 0;
    // SG_INFO("rx = %lf,ry = %lf,px = %lf,py = %lf", road_section_point.x,
    //         road_section_point.y, vehicle_state.position.x,
    //         vehicle_state.position.y);
    routing_msg.distance = longitudinal_s;
    // SG_INFO("dist_sqr = %lf,lgn = %lf,current_road_cnt = %d",
    // abs_distance_sqr,
    //         longitudinal_s, current_road_cnt);

    if (abs_distance_sqr > lane_change_dist_sqr) {
        routing_msg.turn = TURN::STRAIGHT;

    } else {
        routing_msg.turn = ActionTrans(routing_steps_[current_road_cnt].action);
    }

    if (current_road_cnt >= routing_steps_.size() - 1) {
        SG_ERROR("current road cnt may be incorrect");
    } else {
        if (routing_steps_[current_road_cnt + 1].polyline.size() > 1) {
            routing_msg.next_road_ref_points.emplace_back(
                routing_steps_[current_road_cnt + 1].polyline[0]);
            routing_msg.next_road_ref_points.emplace_back(
                routing_steps_[current_road_cnt + 1].polyline[1]);

            double dx = routing_steps_[current_road_cnt + 1].polyline[1].x -
                        routing_steps_[current_road_cnt + 1].polyline[0].x;
            double dy = routing_steps_[current_road_cnt + 1].polyline[1].y -
                        routing_steps_[current_road_cnt + 1].polyline[0].y;
            routing_msg.next_road_theta = std::atan2(dy, dx);
        } else {
            SG_WARN("next road theta may be incorrect");
            routing_msg.next_road_theta = 0;
        }
    }

    double nearest_dist = 1e10;
    int32_t nearest_count = 0;
    for (int32_t i = 0; i < routing_steps_[current_road_cnt].polyline.size();
         ++i) {
        dx = routing_steps_[current_road_cnt].polyline[i].x -
             vehicle_state.position.x;
        dy = routing_steps_[current_road_cnt].polyline[i].y -
             vehicle_state.position.y;

        longitudinal_s = dx * cos_car_yaw + dy * sin_car_yaw;
        if (nearest_dist > std::fabs(longitudinal_s)) {
            nearest_dist = std::fabs(longitudinal_s);
            nearest_count = i;
        }
    }
    if (nearest_count == routing_steps_[current_road_cnt].polyline.size() - 1) {
        --nearest_count;
    }
    dx = routing_steps_[current_road_cnt].polyline[nearest_count + 1].x -
         routing_steps_[current_road_cnt].polyline[nearest_count].x;
    dy = routing_steps_[current_road_cnt].polyline[nearest_count + 1].y -
         routing_steps_[current_road_cnt].polyline[nearest_count].y;
    routing_msg.current_road_theta = std::atan2(dy, dx);

    // SG_INFO("current road = %s",
    // routing_steps_[current_road_cnt].road.c_str());

    // int32_t tmp_x = 4092.525475;
    // int32_t tmp_y = 2072.529407;
    // dx = tmp_x - vehicle_state.position.x;
    // dy = tmp_y - vehicle_state.position.y;
    // longitudinal_s = dx * cos_car_yaw + dy * sin_car_yaw;

    // if (longitudinal_s < 0) {
    //     routing_msg.turn = TURN::LEFT;
    // }
    // SG_INFO("routing turn msg");
    // TurnPrint(routing_msg.turn);

    return true;
}
// bool Routing::GetNextJunctionMsg(const PosFromIns &vehicle_state,
//                                  RoutingMsg &routing_msg) {
//     if (routing_nodes_.empty() == true) {
//         return false;
//     }
//     double dx = routing_nodes_.front().x - vehicle_state.position.x;
//     double dy = routing_nodes_.front().y - vehicle_state.position.y;
//     double cos_car_yaw = std::cos(vehicle_state.yaw);
//     double sin_car_yaw = std::sin(vehicle_state.yaw);
//     double abs_distance_sqr = dx * dx + dy * dy;

//     double longitudinal_s = dx * cos_car_yaw + dy * sin_car_yaw;

//     if (longitudinal_s < 0 && abs_distance_sqr < 9.0) {
//         routing_nodes_.pop_front();
//     }
//     routing_msg.distance = longitudinal_s;
//     routing_msg.turn = routing_nodes_.front().turn;
//     SG_INFO("distance = %lf", routing_msg.distance);
//     TurnPrint(routing_msg.turn);
//     return true;
// }
}  // namespace planning_lib
}  // namespace jarvis