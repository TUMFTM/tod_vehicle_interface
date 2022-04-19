cmake_minimum_required(VERSION 3.0.2)
project(tod_tum-q7_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    tf2
    tf2_ros
    tod_msgs
    tod_network
    tod_helper
    )

catkin_package(
    # INCLUDE_DIRS
    CATKIN_DEPENDS roscpp nav_msgs tf2 tf2_ros tod_msgs tod_network tod_helper
    )

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
