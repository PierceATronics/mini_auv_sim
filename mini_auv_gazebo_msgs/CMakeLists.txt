cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mini_auv_gazebo_msgs)

find_package(Protobuf REQUIRED)

#build all the protobuf messages
file(GLOB msgs msgs/*.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(mini_auv_gazebo_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mini_auv_gazebo_msgs ${PROTOBUF_LIBRARY})
#target_include_directories(mini_auv_gazebo_msgs PUBLIC {CMAKE_CURRENT_SOURCE_DIR})