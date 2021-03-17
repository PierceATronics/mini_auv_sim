#include "PicoSensorHubEmulator.hh"


PicoSensorHubEmulator::PicoSensorHubEmulator(std::string &_model_name){
    
    /*
    //create a serial port and assign it to the class member variable
    std::unique_ptr<serial::Serial> s(new serial::Serial);
    this->port = std::move(s);

    this->model_name = _model_name;
    std::string port_name("/dev/");
    port_name.append(model_name).append("S"); 
    std::cout << port_name << std::endl;
    
    //Open the serial port
    this->port->open(port_name.c_str(), 115200);
    std::cout << "Is there serial port " << port_name << " open?  " << this->port->isOpen() << std::endl;
   
    */
    //Set up the gazebo connections
    this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->node->Init();
    
    this->depth_sub = this->node->Subscribe("~/depth", &PicoSensorHubEmulator::depth_unpack_callback, this);
    this->imu_sub = this->node->Subscribe("~/pico/hull/imu_sensor/imu", 
            &PicoSensorHubEmulator::imu_unpack_callback, this);
}

void PicoSensorHubEmulator::depth_unpack_callback(DoublePtr &depth_msg){

    this->z = depth_msg->data();

}

void PicoSensorHubEmulator::imu_unpack_callback(IMUPtr &imu_msg){
    
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
void PicoSensorHubEmulator::run(){

    while(true){


    }

}




int main(int _argc, char **_argv){
    
    gazebo::client::setup(_argc, _argv);
    
    std::string model_name("pico");
    PicoSensorHubEmulator pico_sensorhub_emulator(model_name);
    pico_sensorhub_emulator.run();
    return(0);

}
/*    
 
    //Send the data via the virtual serial port at a fixed rate.
    if(this->_cnt < this->serial_loop_dt){
                    
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - this->_restart_time);
    
        this->_cnt = dt.count();
    }
    
    //  check serial requests, send needed responses, and reset the timer.
    else{
        //----------PUT SERIAL COMMUNCIATION EMULATION HERE!!!!!-----------//
        
        //read some data if available
        //  ->If a request has been made, process the request
        //  ->If the request is valid, package the data and send the data.
        //----------------------------------------------------------------//
        
        std::vector<uint8_t> some_data = {0x4D, 0x51};
        this->port->transmit(some_data);
        std::cout << "Here" << std::endl;
        
        req_buffer = this->port->receiveAsync(1, 5000);
        std::cout << "did i get past" << std::endl;
        
        std::cout << req_buffer.valid() << std::endl;
        if(req_buffer.get()[0] == 'H') std::cout << "Got good data" << std::endl;
        //std::cout << "Sending some rad sensor data" << this->_cnt <<  '\n';
        this->_restart_time = std::chrono::high_resolution_clock::now();
        this->_cnt = 0.0;
    }
}
*/
