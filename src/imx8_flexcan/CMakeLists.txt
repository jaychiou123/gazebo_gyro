cmake_minimum_required(VERSION 3.0.2)
project(imx8_flexcan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imx8_flexcan
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imx8flexcan_receive src/imx8flexcan_receive.cpp)
target_link_libraries(imx8flexcan_receive ${catkin_LIBRARIES})

add_executable(imx8flexcan_filter src/imx8flexcan_filter.cpp)
target_link_libraries(imx8flexcan_filter ${catkin_LIBRARIES})

add_executable(imx8flexcan_transmit src/imx8flexcan_transmit.cpp)
target_link_libraries(imx8flexcan_transmit ${catkin_LIBRARIES})
