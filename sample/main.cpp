#include <planning/planning_interface.h>
#include <ros/ros.h>
#include <sglog/sglog.h>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "planning/planning_interface.h"
#include "sg_sensor_msgs/Inspva.h"

using namespace jarvis::planning_lib;

std::shared_ptr<jarvis::planning_lib::PlanningInterface> plnning_interface_ptr;

// TrajectoryMsg trajectory_msg;
// ObstaclesInfo obstacles_info;
// LaneInfo lane_info;
// VehicleState vehicle_state;
// ObstaclesInfo obstacles;

// const double semimajor_len = 6378137;
// const double semiminor_len = 6356752.31414;
// const double earth_radius = 6378137;
// const double oblateness = 1 / 298.257222101;
// const double first_eccentricity = 0.0818191910428;

// const double orgin_longitude = 2.074710701656759;
// const double orgin_latitude = 0.5586257075569977;
// const double orgin_altitude = 0.22758597622763593;

// const double orgin_x = -2614020.578497937;
// const double orgin_y = 4740731.728376352;
// const double orgin_z = 3361079.9529776173;

// void Wgs84ToLocalCoord(const double longitude, const double latitude,
//                        const double altitude, double *local_x, double
//                        *local_y, double *local_z) {
//     double earth_radius_p =
//         earth_radius *
//         (1 + oblateness * std::sin(latitude) * std::sin(latitude));
//     double spre_coord_x =
//         (earth_radius_p + altitude) * std::cos(latitude) *
//         std::cos(longitude);
//     double spre_coord_y =
//         (earth_radius_p + altitude) * std::cos(latitude) *
//         std::sin(longitude);
//     double spre_coord_z =
//         ((1 - first_eccentricity * first_eccentricity) * earth_radius_p +
//          altitude) *
//         std::sin(latitude);

//     *local_x = -std::sin(orgin_longitude) * (spre_coord_x - orgin_x) +
//                std::cos(orgin_longitude) * (spre_coord_y - orgin_y);
//     *local_y = -std::sin(orgin_latitude) * std::cos(orgin_longitude) *
//                    (spre_coord_x - orgin_x) -
//                std::sin(orgin_latitude) * std::sin(orgin_longitude) *
//                    (spre_coord_y - orgin_y) +
//                std::cos(orgin_latitude) * (spre_coord_z - orgin_z);
//     *local_z = std::cos(orgin_latitude) * std::cos(orgin_longitude) *
//                    (spre_coord_x - orgin_x) -
//                std::cos(orgin_latitude) * std::sin(orgin_longitude) *
//                    (spre_coord_y - orgin_y) +
//                std::sin(orgin_latitude) * (spre_coord_z - orgin_z);
// }

// inline double AngleToRadian(double yaw_degree) {
//     return yaw_degree * 3.14159265358979323846 / 180;
// }

// void ChatterCallbackInspva(
//     const sg_sensor_msgs::InspvaConstPtr &vehicle_inspva_ptr) {
//     static double pre_enu_yaw = 0;
//     static auto pre_time = ros::Time::now();
//     static double pre_enu_x = 0;
//     static double pre_enu_y = 0;

//     // 1.Convert latitude and longitude coordinates to ENU coordinates.
//     double latitude = AngleToRadian(vehicle_inspva_ptr->latitude);
//     double longtitude = AngleToRadian(vehicle_inspva_ptr->longitude);
//     double altitude = AngleToRadian(vehicle_inspva_ptr->altitude);
//     double enu_x = 0, enu_y = 0, enu_z = 0;

//     Wgs84ToLocalCoord(longtitude, latitude, altitude, &enu_x, &enu_y,
//     &enu_z);

//     vehicle_state.x = enu_x;
//     vehicle_state.y = enu_y;
//     vehicle_state.z = enu_z;

//     // 2. Calculate the yaw, dot_yaw in ENU coordinates.
//     double enu_yaw = AngleToRadian(vehicle_inspva_ptr->yaw) + M_PI / 2;
//     auto time_now = ros::Time::now();

//     double delta_t = (time_now - pre_time).toSec();
//     if (std::abs(delta_t - 0.01) < 0.05) delta_t = 0.01;

//     double dot_enu_y = (enu_y - pre_enu_y) / delta_t;
//     double dot_enu_yaw = (enu_yaw - pre_enu_yaw) / delta_t;

//     vehicle_state.yaw = enu_yaw;

//     vehicle_state.is_update = true;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "planning_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    // ros::Rate loop_rate(10);

    // ros::Subscriber inspva_sub =
    //     nh.subscribe("/sensor/ins/fusion", 5, ChatterCallbackInspva);

    plnning_interface_ptr.reset(
        jarvis::planning_lib::PlanningInterface::CreateInstance());

    SG_INFO("version: %s", plnning_interface_ptr->GetVersion().c_str());

    if (!plnning_interface_ptr->Init()) {
        SG_ERROR("init failed");
        return -1;
    }

    // lane_info.has_laneline_msg = false;
    // while (ros::ok()) {
    //     if (vehicle_state.is_update == true) {
    //         vehicle_state.is_update = false;
    //         plnning_interface_ptr->Execute(lane_info, vehicle_state,
    //         obstacles,
    //                                        &trajectory_msg);
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}
