add_custom_target(copy_cfg ALL COMMAND ${CMAKE_COMMAND} -Drobots_dir=${CMAKE_CURRENT_SOURCE_DIR} -P
                                       ${CMAKE_SOURCE_DIR}/cmake/GenControllerCfg.cmake)
