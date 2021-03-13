#include "PIDController6DOF.hh"
#include <chrono>
#include <vector>
using namespace std;

int main(){

    cout << "Testing functionality of the PIDController class" << '\n';
    
    PIDController6DOF robot_pid_controller;

    robot_pid_controller.set_roll_gains(1.0, 0.0, 0.0);
    robot_pid_controller.set_pitch_gains(1.0, 0.0, 0.0);
    robot_pid_controller.set_yaw_gains(1.0, 0.0, 0.0);   
    robot_pid_controller.set_x_gains(1.0, 0.0, 0.0);
    robot_pid_controller.set_y_gains(1.0, 0.0, 0.0);
    robot_pid_controller.set_z_gains(1.0, 0.0, 0.0);

    robot_pid_controller.print_gains();
   
    robot_pid_controller.set_roll_I_limits(-0.1, 0.1);
   
    robot_pid_controller.print_limits();

    //test out the angle wrapping error function
    auto error_1 = robot_pid_controller.angle_wrap_error(3.14159, -3.14159);
    auto error_2 = robot_pid_controller.angle_wrap_error(1.5707, -3.14159);

    cout << "Angle Error: " << error_1 << endl;
    cout << "Angle Error 2: " << error_2 << endl;

    vector<double> set_pt = {1.54, 1.54, 1.54, 5.0, 4.0, 3.0};
    vector<double> process_pt = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   
    
    chrono::duration<double> dt(0.1);
    vector<double>cmd =  robot_pid_controller.update(set_pt, process_pt, dt);
    
    for(int i=0; i < 6; i++){
        cout << "Commands: " << cmd[i] << endl;
    }

    return 0;
}

