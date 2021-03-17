#include "PicoSensorHubEmulator.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PicoSensorHubEmulator);

void PicoSensorHubEmulator::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    this->model = _parent;
    this->sdf = _sdf;
    
    this->relative_depth = 0.0;
    
    
    
    //  connect update callback to Gazebo world update event.
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PicoSensorHubEmulator::on_update, this));
    
        
    //create a serial port and assign it to the class member variable
    std::unique_ptr<serial::Serial> s(new serial::Serial);
    this->port = std::move(s);

    std::string model_name = this->model->GetName();
    std::string port_name("/dev/");
    port_name.append(this->model->GetName()).append("S"); 
    std::cout << port_name << std::endl;
    
    this->port->open(port_name.c_str(), 115200);
    std::cout << "Is there serial port " << port_name << " open?  " << this->port->isOpen() << std::endl;
    

    this->_cnt = 0.0;
}

void PicoSensorHubEmulator::on_update(){

    //Get the depth from the vehicle
    ignition::math::Pose3d world_pos = this->model->WorldPose();
    relative_depth = surface_pos_z - world_pos.Z();
    
 
    //TODO: Add the emulation of the IMU.
    //
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

