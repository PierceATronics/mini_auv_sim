cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mini_auv_gazebo_plugins)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#set(PROTOBUF_IMPORT_DIRS)
#foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#  if(ITR MATCHES ".*gazebo-[0-9.]+$")
#    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
#  endif()
#endforeach()

#include for generated protobuf messages.
#include_directories(build)

#build all the protobuf messages
#file(GLOB msgs msgs/*.proto)
#protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
#add_library(mini_auv_sim_msgs SHARED ${PROTO_SRCS})
#target_link_libraries(mini_auv_sim_msgs ${PROTOBUF_LIBRARY})

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
