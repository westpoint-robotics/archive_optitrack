
add_executable(${PROJECT_NAME}_mocap_node
  mocap_node.cpp
  mocap_config.cpp
  mocap_datapackets.cpp
  socket.cpp
)
target_link_libraries(${PROJECT_NAME}_mocap_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_mocap_node PROPERTIES
                      OUTPUT_NAME mocap_node PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_mocap_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(broadcast_rigid_transform broadcast_rigid_transform.cpp)
target_link_libraries(broadcast_rigid_transform ${catkin_LIBRARIES})

add_executable(transform_to_coords transform_to_coords.cpp latlon_conversions.cpp)
target_link_libraries(transform_to_coords ${catkin_LIBRARIES})

add_executable(correct_vis_coords correct_vis_coords.cpp)
target_link_libraries(correct_vis_coords ${catkin_LIBRARIES})

add_executable(setpoint_node setpoint_node.cpp)
target_link_libraries(setpoint_node ${catkin_LIBRARIES})
