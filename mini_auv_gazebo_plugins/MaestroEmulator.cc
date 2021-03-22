#include "MaestroEmulator.hh"

MaestroEmulator::MaestroEmulator(std::string &_model_name){
    
    this->model_name = _model_name;
    
    this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->node->Init();

    this->thrust_cmd_pub = this->node->Advertise<mini_auv_gazebo_msgs::msgs::ThrustCmd>(
            "~/thrust_cmd");
    
    //create a serial port and assign it to the class member variable
    std::unique_ptr<serial::Serial> s(new serial::Serial);
    this->port = std::move(s);

    std::string port_name("/dev/");
    port_name.append(model_name).append("/maestro/S");
    std::cout << port_name << std::endl;

    //Open the serial port
    this->port->open(port_name.c_str(), 115200);
    
    std::cout << "Is there serial port " << port_name << " open?  " << this->port->isOpen() << std::endl;


}


void MaestroEmulator::publish_thrust_cmd(std::vector<double> &thrusts){
    
    mini_auv_gazebo_msgs::msgs::ThrustCmd _thrust_cmd;
    
    //It is required to explicitly set all of the thrusts.
    _thrust_cmd.set_thruster1(thrusts[0]);
    _thrust_cmd.set_thruster2(thrusts[1]);
    _thrust_cmd.set_thruster3(thrusts[2]);
    _thrust_cmd.set_thruster4(thrusts[3]);
    _thrust_cmd.set_thruster5(thrusts[4]);
    _thrust_cmd.set_thruster6(thrusts[5]);

    this->thrust_cmd_pub->Publish(_thrust_cmd);

}


void MaestroEmulator::run(){

    std::future<std::vector<uint8_t>> future;
    std::vector<uint8_t> received_data;
    
    uint8_t channel = 1;
    this->state = State::ReadHeader;
    
    uint16_t pulse_width;
    while(true){

        switch(state){
            
            //Wait for the expected header byte.
            case State::ReadHeader :
                future = this->port->receiveAsync(1);
                received_data = future.get();

                if(received_data[0] == 0x84){state = State::ReadChannel;}
                else{break;}
            
            case State::ReadChannel :
                
                future = this->port->receiveAsync(1);
                received_data = future.get();
                
                //invalid channel number...throw away this command.
                if(received_data[0] > (uint8_t)6){state = State::ReadHeader; break;}
                
                channel = received[0];
                state = State::ReadCmd;
            
            //Extract the pulse width command that the maestro would receive.
            case State::ReadCmd :
                
                future = this->port->receiveAsync(2);
                received_data = future.get();
                 
                pulse_width = (received_data[0] & 0x7F) | ((uint16_t)(received_data[1] << 7));
            
                state = State::SetThrust;
            
            //Map the pulsewidth to the correct thrust force.
            case State::SetThrust :
                
                std::cout << "Channel " << channel << "Pulse Width: " << pulse_width << std::endl;
                //TODO: Write a mapping function from pulsewidth to thruster force
                //This might require writing a class to emulate the configurations 
                //of the blue robotics ESC's.
                state = State::ReadHeader;
                break; 


        }

    }

}



int main(int _argc, char **_argv){

    //Connect to gazebo server 
    gazebo::client::setup(_argc, _argv);
    
    MaestroEmulator maestro_emulator("pico");
    return(0);
}

