#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <chrono>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include "Serial.h"

namespace gazebo{

    //  Model plugin for the Pico AUV that emulates the sensor hub by sending data via a virtual serial
    //  port
    class PicoSensorHubEmulator : public ModelPlugin{

        public:
        
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            
            //  Callback function called by the gazebo simulator at each simulation iteration
            void on_update();
            
            
        private:
            
            physics::ModelPtr model;
            sdf::ElementPtr sdf;
            event::ConnectionPtr update_connection;

            //  relative depth (value that would be from physical pressure/depth sensor). 
            double relative_depth;

            //  position of specified surface in world. Default z = 0.0
            double surface_pos_z = 0.0;

           std::chrono::high_resolution_clock::time_point _restart_time;

            double _cnt;
            double serial_loop_dt = 10.0; //Serial looping rate in milliseconds
            
            boost::asio::io_service io;
            std::unique_ptr<serial::Serial> port;
            std::future<std::vector<uint8_t>> req_buffer;
    };

}

