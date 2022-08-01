if (DEFINED ENV{ROBOT})
  if (NOT EXISTS ${robots_dir}/$ENV{ROBOT})
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${robots_dir}/$ENV{ROBOT})
  endif ()
endif ()

file(
  GLOB robot_path
  RELATIVE ${robots_dir}
  ${robots_dir}/*)

set(default_controller_yaml "${robots_dir}/controller.yaml.in")
foreach (robot_dir_content ${robot_path})
  if (IS_DIRECTORY ${robots_dir}/${robot_dir_content} AND NOT EXISTS ${robots_dir}/${robot_dir_content}/controller.yaml)
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${default_controller_yaml}
                            ${robots_dir}/${robot_dir_content}/controller.yaml)
  endif ()
endforeach ()
