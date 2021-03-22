#include "BasicESC.hh"

double BasicESC::map(double pulse_width){

   if(pulse_width < max_reverse_pw || pulse_width > max_forward_pw){

       std::cout << "Error: Pulse width " << pulse_width << "us is outside of BasicESC's range...returning previous thrust value." << std::endl;
        
       return(this->previous_thrust);
   }

   this->previous_thrust = ((float)(pulse_width - stopped_pw) * 200.0) / (max_forward_pw - max_reverse_pw);
   return(this->previous_thrust);


}
