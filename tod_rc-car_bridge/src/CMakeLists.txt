add_executable(VehicleCommandToRCCar VehicleCommandToRCCarNode.cpp)
add_dependencies(VehicleCommandToRCCar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleCommandToRCCar ${catkin_LIBRARIES})

add_executable(VehicleDataFromRCCar VehicleDataFromRCCarNode.cpp)
add_dependencies(VehicleDataFromRCCar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleDataFromRCCar ${catkin_LIBRARIES})

install(TARGETS
    VehicleCommandToRCCar
    VehicleDataFromRCCar
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
