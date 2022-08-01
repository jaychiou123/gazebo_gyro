catkin_add_gtest(controller_test controller_test.cpp)
target_link_libraries(controller_test ${PROJECT_NAME} gtest_main) # *face palm*, I really hate catkin

add_executable(standalone_local_planner standalone_local_planner.cpp)
target_link_libraries(standalone_local_planner PRIVATE ${PROJECT_NAME})

add_library(dummy_global_planner utility/dummy_global_planner/dummy_global_planner.cpp)
target_link_libraries(dummy_global_planner PRIVATE catkin_pkg)

find_package(rostest REQUIRED)
add_executable(functionality_test functionality_test.cpp)
target_link_libraries(functionality_test PRIVATE ${PROJECT_NAME} gtest)
target_include_directories(functionality_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB robots_dir_content ${CMAKE_SOURCE_DIR}/robots/*)
foreach (element IN LISTS robots_dir_content)
  if (IS_DIRECTORY ${element})
    set(WORKING_FIELD_DATA ${element}/working_field)
    if (EXISTS ${WORKING_FIELD_DATA}_sim.world AND EXISTS ${WORKING_FIELD_DATA}.png AND EXISTS ${WORKING_FIELD_DATA}.yaml)
      get_filename_component(robot_dir ${element} NAME)
      add_rostest(functionality.test ARGS robot:=${robot_dir})
    endif ()
  endif ()
endforeach ()

# add_rostest_gtest(interface_test interface.test interface_test.cpp)
# target_link_libraries(interface_test PRIVATE ${PROJECT_NAME})
# target_include_directories(interface_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
