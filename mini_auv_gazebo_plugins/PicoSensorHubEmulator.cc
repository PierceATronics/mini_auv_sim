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
    
    //spawn a thread for the com loop
    this->com_loop_running = true;
    com_loop_rate = std::chrono::duration<double>(0.010);
    std::thread com_thread (std::bind(&PicoSensorHubEmulator::com_loop, this));
}

void PicoSensorHubEmulator::on_update(){

    //Get the depth from the vehicle
    ignition::math::Pose3d world_pos = this->model->WorldPose();
    relative_depth = surface_pos_z - world_pos.Z();
    
    //TODO: Add the emulation of the IMU.
}

void PicoSensorHubEmulator::com_loop(){
    
     
    //wait for requests for data, send sensor data upon a valid request.
    while(com_loop_running){
        
        //Start the timer for consistent looping rate
         auto start_time = std::chrono::high_resolution_clock::now();

        //do stuff
        //
        std::cout << "Sending some datas" << '\n';

        //time accumulated since start time.
        auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time);
        
        //Busy while loop to get good time resolution.
        while(dt.count() < com_loop_rate.count()){
             auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time);
      
        }
            
    }
}
