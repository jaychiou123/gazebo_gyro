add_library(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

set_target_properties(${PROJECT_NAME} PROPERTIES INCLUDE_DIRECTORIES "") # to get rid of those "include_directories"
target_link_libraries(${PROJECT_NAME} PUBLIC catkin_pkg Eigen3::Eigen sanitizer_options PRIVATE project_warning)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
