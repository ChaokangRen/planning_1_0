cmake_minimum_required(VERSION 3.0.2)
project(curvaturegenerator)

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
)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
## Build

add_library(${PROJECT_NAME} STATIC
    local_path_generator.cpp
    State/LaneFollow.cpp
    State/InterJunction.cpp
    State/OnJunction.cpp
    State/AwayJunction.cpp
    local_path_common.cpp
    center_line_generator.cpp
)


target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    common
)