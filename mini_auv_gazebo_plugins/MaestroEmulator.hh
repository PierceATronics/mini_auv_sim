#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Quaternion.hh>
#include "mini_auv_gazebo_msgs/ThrustCmd.pb.h"
#include <thread>
#include <chrono>
#include "Serial.h"
#include "BasicESC.hh"

typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::ThrustCmd> ThrustCmdPtr;

enum class State {ReadHeader, ReadChannel, ReadCmd, SetThrust};

class MaestroEmulator{


    public:
        
        MaestroEmulator(std::string &_model_name);
        
        //Main loop receives serial requests to emulate Maestro TTL serial
        //Receives the commands in the Compact Protocol format.
        void run();
    

    private:
        
        gazebo::transport::NodePtr node;
        gazebo::transport::PublisherPtr thrust_cmd_pub;
        State state;

        //Serial port connection
        std::unique_ptr<serial::Serial> port;
        
        std::string model_name;
        
        //package the thrusts into protobuf and publish.
        void publish_thrust_cmd(std::vector<double> &thrusts);
        
        std::vector<double> thrust_state;

        std::unique_ptr<BasicESC> esc;

};

