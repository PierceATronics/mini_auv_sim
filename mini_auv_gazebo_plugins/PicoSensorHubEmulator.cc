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
   
    //Attempt to connect to the serial port...thow exception if cannot connect
    //the serial port shall be open prior to starting up the vehicle.
    //Create the port and assign it to the class member variable. 
    std::unique_ptr<boost::asio::serial_port> temp_port(new boost::asio::serial_port(this->io));
    this->port = std::move(temp_port);
    
    //open the port
    //TODO: Get the port name from model name at load.
    std::cout << "Waiting to connect to port /dev/pts/5...make sure to open port." << std::endl;
    
    boost::system::error_code ec;
    this->port->open("/dev/pts/5", ec);
    std::cout << "Is the serial port open? " <<  this->port->is_open() << std::endl;

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
        
        //----------------------------------------------------------------//

        //std::cout << "Sending some rad sensor data" << this->_cnt <<  '\n';
        this->_restart_time = std::chrono::high_resolution_clock::now();
        this->_cnt = 0.0;
    }
}

