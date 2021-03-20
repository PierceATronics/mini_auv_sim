cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mini_auv_gazebo_plugins)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ThrusterPlugin SHARED ThrusterPlugin.cc)
target_link_libraries(ThrusterPlugin ${GAZEBO_LIBRARIES} mini_auv_gazebo_msgs)
#Make sure to build the mini_auv_gazebo_msgs first
add_dependencies(ThrusterPlugin mini_auv_gazebo_msgs)

add_library(DepthSensorPlugin SHARED DepthSensorPlugin.cc)
target_link_libraries(DepthSensorPlugin ${GAZEBO_LIBRARIES} mini_auv_gazebo_msgs)
add_dependencies(DepthSensorPlugin mini_auv_gazebo_msgs)


add_executable(PicoSensorHubEmulator PicoSensorHubEmulator.cc)
target_link_libraries(PicoSensorHubEmulator ${GAZEBO_LIBRARIES} mini_auv_gazebo_msgs)
#target_include_directories(PicoSensorHubEmulator ${CMAKE_CURRENT_SOURCE_DIR}

add_executable(ThrusterPluginTEST ThrusterPluginTEST.cc)
add_dependencies(ThrusterPluginTEST mini_auv_gazebo_msgs)
target_link_libraries(ThrusterPluginTEST ${GAZEBO_LIBRARIES} mini_auv_gazebo_msgs)
