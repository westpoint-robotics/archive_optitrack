cmake_minimum_required(VERSION 2.8.3)
project(mocap_optitrack)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf roslaunch message_generation)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Check the validity of the supplied launch files.
roslaunch_add_file_check(launch)

add_executable(takeoff src/takeoff.cpp)
target_link_libraries(takeoff ${catkin_LIBRARIES})

add_executable(correct_vis_coords_test src/correct_vis_coords_test.cpp)
target_link_libraries(correct_vis_coords_test ${catkin_LIBRARIES})

add_executable(script_test src/script_test.cpp src/setpoint_scripts.cpp)
target_link_libraries(script_test ${catkin_LIBRARIES})

add_executable(multiquad_script_test src/multiquad_script_test.cpp src/Quad.cpp src/QuadScripts.cpp)
target_link_libraries(multiquad_script_test ${catkin_LIBRARIES})
