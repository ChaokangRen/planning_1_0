cmake_minimum_required(VERSION 3.0.2)
project(emplanner)


find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
    jsoncpp
)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/planning
    ${CMAKE_SOURCE_DIR}/thirdparty/Eigen
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/reference_line
    ${CMAKE_SOURCE_DIR}/src/lane
    ${CMAKE_SOURCE_DIR}/src/decider
)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
## Build

add_library(${PROJECT_NAME} STATIC
    path_optimizer/dp_curve_cost.cpp
    path_optimizer/dp_path_optimizer.cpp
    path_optimizer/qp_path_optimizer.cpp
    speed_optimizer/discretized_path.cpp
    emplanner.cpp
    speed_optimizer/dp_speed_optimizer.cpp
    speed_optimizer/dp_st_graph.cpp 
    speed_optimizer/qp_speed_optimizer.cpp
    speed_optimizer/st_boundary_mapper.cpp
    speed_optimizer/st_point.cpp
    speed_optimizer/st_graph_point.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    common
    referenceline
    lane
    decider
)