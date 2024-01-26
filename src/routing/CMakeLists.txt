cmake_minimum_required(VERSION 3.0.2)
project(routing)

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
    routing.cpp
)


target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    common
)