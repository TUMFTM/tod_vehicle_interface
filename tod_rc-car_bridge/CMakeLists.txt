cmake_minimum_required(VERSION 2.8.3)
project(tod_rc-car_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs nav_msgs ackermann_msgs tod_msgs)

catkin_package(
    INCLUDE_DIRS # include
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs ackermann_msgs tod_msgs)

include_directories(${catkin_INCLUDE_DIRS})

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
