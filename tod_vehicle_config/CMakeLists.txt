cmake_minimum_required(VERSION 2.8.3)
project(tod_vehicle_config)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    INCLUDE_DIRS # include
    #  LIBRARIES tod_vehicle_config
    CATKIN_DEPENDS # roscpp
    #  DEPENDS system_lib
    )

install(DIRECTORY
    vehicle_config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
