cmake_minimum_required(VERSION 3.0.2)
project(tod_vehicle_sim)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp nav_msgs tf2 tod_msgs tod_helper tod_core)

catkin_package(
    INCLUDE_DIRS # include
    CATKIN_DEPENDS roscpp nav_msgs tf2 tod_msgs tod_helper
    )

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