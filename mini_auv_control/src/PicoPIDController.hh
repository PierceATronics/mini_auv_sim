#pragma once

#include <Eigen/Dense>
#include <chrono>
#include "PIDController6DOF.hh"


#include "PIDControl.hh"


typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

//  PID controller for controlling Pico the mini auv.
class PicoPIDController : public PIDController6DOF{
    
    public:
        
        PicoPIDController();
        ~PicoPIDController();
        
        Vector6d update(std::vector<double> &set_pt, std::vector<double> &process_pt, std::chrono::duration<double> &_dt);
        
    private:
        // Matrix to map the pid controller commands to each thruster.
        Matrix6d pid_thruster_mapper;

};
