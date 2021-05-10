## Vehicle Test Node
add_executable(VehicleSimNode VehicleSimNode.cpp VehicleModel.cpp)
add_dependencies(VehicleSimNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(VehicleSimNode ${catkin_LIBRARIES})

install(TARGETS
    VehicleSimNode
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

