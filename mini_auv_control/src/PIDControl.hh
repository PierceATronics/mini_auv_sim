#pragma once
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <chrono>

#include "mini_auv_gazebo_msgs/Double.pb.h"
#include "mini_auv_gazebo_msgs/ThrustCmd.pb.h"
#include "ignition/math/PID.hh"
#include "ignition/math/Quaternion.hh"

#include <Eigen/Dense>

typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::Double> DoublePtr;
typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::ThrustCmd> ThrustCmdPtr;
typedef const boost::shared_ptr<const gazebo::msgs::IMU> IMUPtr;

//  Callback func for the depth data subscriber.
void depth_unpack_callback(DoublePtr &depth_msg);

//  Callback func for the imu data subscriber.
void imu_unpack_callback(IMUPtr &imu_msg);
