#pragma once
#include <iostream>

//  Contains specification for mapping the pusle width received via PWM to output thrust percentage.
class BasicESC {

    public:

        //Maps the pules width from ESC driver to a value from -100 to 100.
        //-100 is max reverse thrust
        //100 is max forward thrust
        //0 the motor is stopped.
        double map(double pulse_width);
    
    private:

        uint16_t max_reverse_pw = 1100; //1100us
        uint16_t max_forward_pw = 1900; //1900us
        uint16_t stopped_pw = 1500;  //1500us
        double previous_thrust = 0.0;

};

