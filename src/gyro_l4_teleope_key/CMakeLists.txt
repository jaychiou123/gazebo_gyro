cmake_minimum_required(VERSION 3.0.2)
project(gyro_l4_teleope_key)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gyro_l4_teleope_key
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/gyro_l4_teleope_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)