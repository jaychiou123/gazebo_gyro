cmake_minimum_required(VERSION 3.5.0)
project(tony_local_planner VERSION 1.0.0) # see Professional CMake, P.181, consider adopting this

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(
  catkin REQUIRED
  COMPONENTS costmap_2d
             dynamic_reconfigure
             geometry_msgs
             nav_core
             nav_msgs
             pluginlib
             actionlib
             roscpp
             rosbag
             tf)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(StaticAnalyzers)
include(StandardProjectSetting)

add_library(catkin_pkg INTERFACE)
target_include_directories(catkin_pkg SYSTEM INTERFACE ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
target_include_directories(catkin_pkg INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(catkin_pkg INTERFACE ${catkin_LIBRARIES})

## System dependencies are found with CMake's conventions
set(EIGEN_MINIMUM_VERSION "3.3.1")
find_package(Eigen3 ${EIGEN_MINIMUM_VERSION} REQUIRED NO_MODULE)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(cfg/TonyLocalPlanner.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    nav_msgs
    std_msgs
    pluginlib
    roscpp
    tf
    #  DEPENDS Eigen3
)

###########
## Build ##
###########

include(CompilerWarning)
add_library(project_warning INTERFACE)
set_project_warnings(project_warning)

include(Sanitizers)
add_library(sanitizer_options INTERFACE)
enable_sanitizers(sanitizer_options)

add_subdirectory(robots)
add_subdirectory(src)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS scripts/add_sim_map.py scripts/run_local_planner_test.sh scripts/start_gain_tuning.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn"
                                                                                                                                  EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES ${PROJECT_NAME}_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif ()
