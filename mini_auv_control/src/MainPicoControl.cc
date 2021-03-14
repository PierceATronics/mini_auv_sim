#include "PicoPIDController.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Quaternion.hh>

#include "mini_auv_gazebo_msgs/Double.pb.h"
#include "mini_auv_gazebo_msgs/ThrustCmd.pb.h"

typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::Double> DoublePtr;
typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::ThrustCmd> ThrustCmdPtr;
typedef const boost::shared_ptr<const gazebo::msgs::IMU> IMUPtr;

//  Callback func for the depth data subscriber.
void depth_unpack_callback(DoublePtr &depth_msg);

//  Callback func for the imu data subscriber.
void imu_unpack_callback(IMUPtr &imu_msg);

void initialize_pid_controller(PicoPIDController *controller);

//global variables
double depth = 0.0;
double roll = 0.0, pitch = 0.0, yaw = 0.0, x = 0.0, y = 0.0, z = 0.0;
std::chrono::duration<double> dt(0.010);

//Set point is the desired position of the vehicle
std::vector<double> set_pt = {0.0, 0.0, 0.0, 0.0, 0.0, 3.0}; // r p yaw x y z

int main(int _argc, char **_argv){
    
    gazebo::client::setup(_argc, _argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    //  Depth data subscriber
    gazebo::transport::SubscriberPtr depth_sub = node->Subscribe(
        "~/depth",
        &depth_unpack_callback);

    //  IMU sensor subscriber
    gazebo::transport::SubscriberPtr imu_sub = node->Subscribe(
        "~/pico/hull/imu_sensor/imu",
        &imu_unpack_callback);

    //  Thrust CMD publisher
    gazebo::transport::PublisherPtr thrust_cmd_pub = 
        node->Advertise<mini_auv_gazebo_msgs::msgs::ThrustCmd>("~/thrust_cmd");
    
    //Initialize the thruster msg. 
    mini_auv_gazebo_msgs::msgs::ThrustCmd thrust_cmd;
    thrust_cmd.set_thruster1(0.0);
    thrust_cmd.set_thruster2(0.0);
    thrust_cmd.set_thruster3(0.0);
    thrust_cmd.set_thruster4(0.0);
    thrust_cmd.set_thruster5(0.0);
    thrust_cmd.set_thruster6(0.0);

    PicoPIDController pico_pid_controller;
    initialize_pid_controller(&pico_pid_controller); 
    
    //Run the control loop 
    while(true){
        

        std::vector<double> process_pt = {roll, pitch, yaw, x, y, z};
        
        Vector6d thrusts = pico_pid_controller.update(set_pt, process_pt, dt);
        
        //package the thrusts values into the protobuf message.
        thrust_cmd.set_thruster1(thrusts[0]);
        thrust_cmd.set_thruster2(thrusts[1]);
        thrust_cmd.set_thruster3(thrusts[2]);
        thrust_cmd.set_thruster4(thrusts[3]);
        thrust_cmd.set_thruster5(thrusts[4]);
        thrust_cmd.set_thruster6(thrusts[5]);

        //publish the thrusts to the robot in Gazebo 
        thrust_cmd_pub->Publish(thrust_cmd);
        gazebo::common::Time::MSleep(10);
    }

    return 0;

}

void initialize_pid_controller(PicoPIDController *controller){
    
    controller->set_roll_gains(0.0, 0.0, 0.0);
    controller->set_roll_I_limits(-0.1, 0.1);

    controller->set_pitch_gains(0.0, 0.0, 0.0);
    controller->set_pitch_I_limits(-0.1, 0.1);
}

void depth_unpack_callback(DoublePtr &depth_msg){

    z = depth_msg->data();
}

void imu_unpack_callback(IMUPtr &imu_msg){
    auto qx = imu_msg->orientation().x();
    auto qy = imu_msg->orientation().y();
    auto qz = imu_msg->orientation().z();
    auto qw = imu_msg->orientation().w();

    //  Cvt quaternion to Euler (roll, pitch ,yaw)
    ignition::math::Quaternion Q(qw, qx, qy, qz);

    roll = Q.Roll();
    pitch = Q.Pitch();
    //  TODO: THIS MAY NOT BE THE CORRECT YAW: USE MAGNETOMETER.
    yaw = Q.Yaw();

    std::cout << "Orientation --" << '\n';
    std::cout << "\tRoll:  " << roll << '\n';
    std::cout << "\tPitch: " << pitch << '\n';
    std::cout << "\tYaw:   " << yaw << '\n';
    std::cout << "Depth: " << z << '\n';
}
