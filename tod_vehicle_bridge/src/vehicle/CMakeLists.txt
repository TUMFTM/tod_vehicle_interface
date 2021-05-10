set(TARGETS ${TARGETS} ControlCommandMultiplexer)
add_executable(ControlCommandMultiplexer ControlCommandMultiplexerNode.cpp)
target_link_libraries(ControlCommandMultiplexer ${catkin_LIBRARIES})
target_include_directories(ControlCommandMultiplexer PRIVATE ${catkin_INCLUDE_DIRS})

set(TARGETS ${TARGETS} VehicleDataSender)
add_executable(VehicleDataSender VehicleDataSender.cpp)
target_link_libraries(VehicleDataSender ${catkin_LIBRARIES})
target_include_directories(VehicleDataSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET VehicleDataSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} GPSSender)
add_executable(GPSSender GPSSender.cpp)
target_link_libraries(GPSSender ${catkin_LIBRARIES})
target_include_directories(GPSSender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET GPSSender PROPERTY CXX_STANDARD 17)

set(TARGETS ${TARGETS} OdometrySender)
add_executable(OdometrySender OdometrySender.cpp)
target_link_libraries(OdometrySender ${catkin_LIBRARIES})
target_include_directories(OdometrySender PRIVATE ${catkin_INCLUDE_DIRS})
set_property(TARGET OdometrySender PROPERTY CXX_STANDARD 17)

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
