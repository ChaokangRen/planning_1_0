cmake_minimum_required(VERSION 3.0.2)
project(common)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
    jsoncpp
)

message("depend inc: ${PC_DEPEND_INCLUDE_DIRS}")
message("depend lib: ${PC_DEPEND_LIBRARIES}")

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/planning
    ${CMAKE_SOURCE_DIR}/thirdparty/Eigen
    ${PC_DEPEND_INCLUDE_DIRS}
)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
## Build

add_library(${PROJECT_NAME} STATIC
    common.cpp
    qpoases_solver.cpp
    vec2d.cpp
    quintic_polynomial_curve.cpp
    line_segment2d.cpp
    box2d.cpp
    polygon2d.cpp
    cartesian_frenet_conversion.cpp
    lane_line.cpp
    speed_data.cpp
    obstacle.cpp
    virtual_obs.cpp
    obstacle_container.cpp
    json_parser.cpp
    st_boundary.cpp
)


target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
    qpOASES
)