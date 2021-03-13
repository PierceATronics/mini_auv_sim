cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mini_auv_sim)

include_directories(build)
add_subdirectory(mini_auv_gazebo_msgs)
add_subdirectory(mini_auv_gazebo_plugins)
add_subdirectory(mini_auv_control)
