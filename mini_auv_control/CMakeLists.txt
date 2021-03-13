cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mini_auv_control)

find_package(gazebo REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(ignition-math6 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(PIDControl src/PIDControl.cc)
target_link_libraries(PIDControl ${GAZEBO_LIBRARIES} mini_auv_gazebo_msgs Eigen3::Eigen)
add_dependencies(PIDControl mini_auv_gazebo_msgs)

add_library(PIDController6DOF src/PIDController6DOF.cc)
target_link_libraries(PIDController6DOF Eigen3::Eigen ignition-math6::ignition-math6)

add_executable(PIDController6DOF_TEST src/PIDController6DOF_TEST.cc)
target_link_libraries(PIDController6DOF_TEST PIDController6DOF)

add_library(PicoPIDController src/PicoPIDController.cc)
target_link_libraries(PicoPIDController Eigen3::Eigen PIDController6DOF)
