include_directories(${catkin_INCLUDE_DIRS})

add_executable(VehicleDataFromAutobox VehicleDataFromAutoboxNode.cpp)
add_dependencies(VehicleDataFromAutobox ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleDataFromAutobox ${catkin_LIBRARIES})

add_executable(SafetyDriverStatusFromAutobox SafetyDriverStatusFromAutoboxNode.cpp)
add_dependencies(SafetyDriverStatusFromAutobox ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(SafetyDriverStatusFromAutobox ${catkin_LIBRARIES})

add_executable(ControlCommandToAutobox ControlCommandToAutoboxNode.cpp)
add_dependencies(ControlCommandToAutobox ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ControlCommandToAutobox ${catkin_LIBRARIES})

install(TARGETS
    VehicleDataFromAutobox
    SafetyDriverStatusFromAutobox
    ControlCommandToAutobox
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

