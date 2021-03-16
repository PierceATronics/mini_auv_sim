#ifndef PicoSENSOR_HUB_EMULATOR_H
#define PicoSENSOR_HUB_EMULATOR_H


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include <chrono>

namespace gazebo{

    //  Model plugin for the Pico AUV that emulates the sensor hub by sending data via a virtual serial
    //  port
    class PicoSensorHubEmulator : public ModelPlugin{

        public:
        
            PicoSensorHubEmulator();

            ~PicoSensorHubEmulator();

            virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            
            //  Callback function called by the gazebo simulator at each simulation iteration
            void on_update();
            
            //  communication loop that reads request and gives responses through the virtual
            //  serial port
            void com_loop();


        private:
            
            physics::ModelPtr model;
            sdf::ElementPtr sdf;
            event::ConnectionPtr update_connection;

            //  relative depth (value that would be from physical pressure/depth sensor). 
            double relative_depth;

            //  position of specified surface in world. Default z = 0.0
            double surface_pos_z = 0.0;

            bool com_loop_running;
            std::chrono::duration<double> com_loop_rate;
    };

}



#endif
