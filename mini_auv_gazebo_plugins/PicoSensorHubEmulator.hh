#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Quaternion.hh>
#include "mini_auv_gazebo_msgs/Double.pb.h"
#include "mini_auv_gazebo_msgs/ThrustCmd.pb.h"
#include <thread>
#include <chrono>
#include "Serial.h"

typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::Double> DoublePtr;
typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::ThrustCmd> ThrustCmdPtr;
typedef const boost::shared_ptr<const gazebo::msgs::IMU> IMUPtr;

//Used to conveintely pack floats into 4bytes for serial transmission.
union float_char{
    
    float f;
    uint8_t c[4];

};

class PicoSensorHubEmulator{

    public:
        
        PicoSensorHubEmulator(std::string &_model_name);
        
        //Main loop receives serial requests and gives response (of sensor data);
        void run(); 
            
    private:
       
        //Set up the transport to get data from the model in the gazebo simulator
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr depth_sub;
        gazebo::transport::SubscriberPtr imu_sub;
        
        double roll, pitch, yaw, x, y, z;
        
        const uint8_t HEADER_BYTE = 0xA4;
        const uint8_t DEPTH_CMD = 0x02;
        const uint8_t IMU_CMD = 0x03;
        const uint8_t ALL_SENSOR_CMD = 0x04;
        const uint8_t END_BYTE    = 0xA0;

        std::string model_name;
        std::chrono::high_resolution_clock::time_point _restart_time;

        double serial_loop_dt = 10.0; //Serial looping rate in milliseconds
        
        //Serial port connection
        std::unique_ptr<serial::Serial> port;

        //  Callback func for the depth data subscriber.
        void depth_unpack_callback(DoublePtr &depth_msg);

        //  Callback func for the imu data subscriber.
        void imu_unpack_callback(IMUPtr &imu_msg);
        
        void send_depth_data();
        
        void send_imu_data();

        void send_all_sensor_data();
        
        std::vector<uint8_t> pack_floats(std::vector<float> &f);
};


