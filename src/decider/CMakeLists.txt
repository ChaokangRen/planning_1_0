cmake_minimum_required(VERSION 3.0.2)
project(decider)


find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
)
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/planning
    ${CMAKE_SOURCE_DIR}/thirdparty/Eigen
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/traffic_light
    ${PC_DEPEND_INCLUDE_DIRS}
)

## Build

add_library(${PROJECT_NAME} STATIC
    decider.cpp
    lane_borrow_decider.cpp
    path_bounds_decider.cpp
    lane_change_decider.cpp
    lateral_obstacle_decider.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    common
)