cmake_minimum_required(VERSION 3.10)
project(planning VERSION 0.2.0)

set(CMAKE_INSTALL_PREFIX /usr)

## set version
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}-${CPACK_PACKAGE_VERSION}")
add_definitions(-DPKG_VERSION="${CPACK_PACKAGE_VERSION}")

## Build
add_compile_options(-fPIC)

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    sglog
    sgtime
)

message("depend inc: ${PC_DEPEND_INCLUDE_DIRS}")
message("depend lib: ${PC_DEPEND_LIBRARIES}")

include_directories(
    include
    ${PC_DEPEND_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/thirdparty/Eigen
    ${CMAKE_SOURCE_DIR}/include/planning
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/reference_line_smooth
    ${CMAKE_SOURCE_DIR}/src/reference_line
    # ${CMAKE_SOURCE_DIR}/src/prediction
    ${CMAKE_SOURCE_DIR}/src/emplanner
    ${CMAKE_SOURCE_DIR}/src/emplanner/path_optimizer
    ${CMAKE_SOURCE_DIR}/src/emplanner/speed_optimizer
    ${CMAKE_SOURCE_DIR}/src/emplanner/rtk_generator
    ${CMAKE_SOURCE_DIR}/src/traffic_light
    ${CMAKE_SOURCE_DIR}/src/lane
    ${CMAKE_SOURCE_DIR}/src/local_path_generator
    ${CMAKE_SOURCE_DIR}/src/local_path_generator/state
    ${CMAKE_SOURCE_DIR}/src/routing
    ${CMAKE_SOURCE_DIR}/src/decider
)

# add_subdirectory(sample)
add_subdirectory(src/common)
add_subdirectory(src/reference_line_smooth)
add_subdirectory(src/reference_line)
add_subdirectory(src/emplanner)
add_subdirectory(src/rtk_generator)
# add_subdirectory(src/prediction)
add_subdirectory(src/traffic_light)
add_subdirectory(src/lane)
add_subdirectory(src/local_path_generator)
add_subdirectory(src/routing)
add_subdirectory(src/decider)

add_library(${PROJECT_NAME} SHARED
    src/planning_component.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
    common
    referencelinesmooth
    referenceline
    emplanner
    simulator
    # prediction
    traffic_light
    lane
    curvaturegenerator
    routing
    decider
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Install
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
    COMPONENT core
    NAMELINK_COMPONENT devel
)

install(DIRECTORY src/resource
    COMPONENT core
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jarvis/${PROJECT_NAME}
)
install(DIRECTORY include/
    COMPONENT devel
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/jarvis
)

install(DIRECTORY sample
    COMPONENT devel
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jarvis/${PROJECT_NAME}
    PATTERN "build" EXCLUDE
)

configure_file(planning.pc.in planning.pc)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/planning.pc
    COMPONENT devel
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/pkgconfig
)

## Packages
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VENDOR "SingGo")
set(CPACK_PACKAGE_CONTACT "renchaokang@sg.cambricon.com")
set(CPACK_GENERATOR "DEB")

set(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)
set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "jarvis-framework-sdk, libeigen3-dev")
set(CPACK_DEB_COMPONENT_INSTALL ON)

# core pkg
set(CPACK_DEBIAN_CORE_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_CORE_DEBUGINFO_PACKAGE ON)
set(CPACK_DEBIAN_CORE_PACKAGE_NAME ${CPACK_PACKAGE_NAME})
# devel pkg
set(CPACK_DEBIAN_DEVEL_PACKAGE_DEPENDS "${CPACK_PACKAGE_NAME} (= ${CPACK_PACKAGE_VERSION})")

set(CPACK_SET_DESTDIR true)
set(CPACK_INSTALL_PREFIX /opt/ros/$ENV{ROS_DISTRO})

include(CPack)


