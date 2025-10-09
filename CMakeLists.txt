cmake_minimum_required(VERSION 3.16)

project(erl_geometry_rviz_plugin #
    LANGUAGES CXX #
    VERSION 0.1.0 #
    DESCRIPTION "erl_geometry_rviz_plugin is a ROS package for visualizing erl_geometry_msgs.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if(NOT COMMAND erl_project_setup)
    find_package(erl_cmake_tools REQUIRED)
endif()

erl_project_setup(ERL_PACKAGES erl_common erl_geometry)
include(cmake/erl_geometry_rviz_plugin_dependencies.cmake)
erl_setup_ros(
    CATKIN_COMPONENTS rviz erl_geometry_msgs
    CATKIN_DEPENDS rviz erl_geometry_msgs
    LIBRARIES ${PROJECT_NAME}
    ROS2_COMPONENTS pluginlib rviz_common rviz_default_plugins erl_geometry_msgs
    CFG_EXTRAS erl_geometry_rviz_plugin_dependencies.cmake)
erl_add_ros_src()
erl_add_tests(LIBRARIES Open3D::Open3D)
erl_install(
    LIBRARIES ${${PROJECT_NAME}_COLLECTED_LIBRARIES}
    EXECUTABLES ${${PROJECT_NAME}_COLLECTED_EXECUTABLES})
erl_mark_project_found()
