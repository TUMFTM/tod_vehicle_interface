set(TARGETS ${TARGETS} VehicleDataReceiver)
add_executable(VehicleDataReceiver VehicleDataReceiver.cpp)
target_link_libraries(VehicleDataReceiver ${catkin_LIBRARIES})
target_include_directories(VehicleDataReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET VehicleDataReceiver PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} GPSReceiver)
add_executable(GPSReceiver GPSReceiver.cpp)
target_link_libraries(GPSReceiver ${catkin_LIBRARIES})
target_include_directories(GPSReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET GPSReceiver PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} OdometryReceiver)
add_executable(OdometryReceiver OdometryReceiver.cpp)
target_link_libraries(OdometryReceiver ${catkin_LIBRARIES})
target_include_directories(OdometryReceiver PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET OdometryReceiver PROPERTY CXX_STANDARD 17)


install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
