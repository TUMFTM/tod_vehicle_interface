add_executable(VehicleCommandToLGSVL VehicleCommandToLGSVL.cpp)
add_dependencies(VehicleCommandToLGSVL ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleCommandToLGSVL ${catkin_LIBRARIES})

add_executable(VehicleDataFromLGSVL VehicleDataFromLGSVL.cpp)
add_dependencies(VehicleDataFromLGSVL ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleDataFromLGSVL ${catkin_LIBRARIES})

add_executable(SafetyDriverStatusPublisher SafetyDriverStatusPublisher.cpp)
add_dependencies(SafetyDriverStatusPublisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(SafetyDriverStatusPublisher ${catkin_LIBRARIES})

add_executable(OdometryOffset OdometryOffsetNode.cpp)
add_dependencies(OdometryOffset ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(OdometryOffset ${catkin_LIBRARIES})

install(TARGETS
    VehicleCommandToLGSVL
    VehicleDataFromLGSVL
    SafetyDriverStatusPublisher
    OdometryOffset
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
