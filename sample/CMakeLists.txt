cmake_minimum_required(VERSION 3.10)
project(planning_sample)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        rosbag
        sg_vehicle_msgs
)

catkin_package()

###########
## Build ##
###########

find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND REQUIRED
    sglog
    planning
)

message("depend inc: ${PC_DEPEND_INCLUDE_DIRS}")
message("depend lib: ${PC_DEPEND_LIBRARIES}")
message("depend lib_dir: ${PC_DEPEND_LIBRARY_DIRS}")


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PC_DEPEND_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/rtk_generator
    ${CMAKE_SOURCE_DIR}/thirdparty/Eigen
)

add_executable(${PROJECT_NAME}
    main.cpp
)

link_directories(${PC_DEPEND_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PC_DEPEND_LIBRARIES}
    pthread
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
)

install(FILES package.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)