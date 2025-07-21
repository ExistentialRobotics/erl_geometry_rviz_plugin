message(STATUS "ROS2 activated, building ROS2 stuff")

# library: erl_geometry_rviz_plugin
file(GLOB RVIZ_PLUGIN_INCLUDE_FILES include/erl_geometry_rviz_plugin/ros2/*.hpp)
file(GLOB RVIZ_PLUGIN_SRC_FILES src/ros2/*.cpp)
add_library(${PROJECT_NAME} MODULE ${RVIZ_PLUGIN_INCLUDE_FILES} ${RVIZ_PLUGIN_SRC_FILES})
erl_collect_targets(LIBRARIES ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIR}>
                                                  $<INSTALL_INTERFACE:${${PROJECT_NAME}_INSTALL_INCLUDE_DIR}>)
erl_target_dependencies(${PROJECT_NAME} PRIVATE ${QT_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)
set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC ON)

# Avoid OGRE deprecation warnings under C++17
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-register")
