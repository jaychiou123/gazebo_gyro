cmake_minimum_required(VERSION 3.0.2)
project(tf_move_relative)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  dynamic_reconfigure
  geometry_msgs
  move_base_msgs
  roscpp
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/tf_move_relative.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    geometry_msgs
    move_base_msgs
    roscpp
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_move_relative src/tf_move_relative_server.cpp src/tf_move_relative.cpp)
target_link_libraries(tf_move_relative ${catkin_LIBRARIES})
add_dependencies(tf_move_relative tf_move_relative_gencfg)

add_executable(tf_move_relative_client src/tf_move_relative_client.cpp)
target_link_libraries(tf_move_relative_client ${catkin_LIBRARIES})

add_executable(turtle_listener src/turtle_listener.cpp)
target_link_libraries(turtle_listener ${catkin_LIBRARIES})