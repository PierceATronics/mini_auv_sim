#include "PIDControl.hh"

double depth = 0.0;
double roll = 0.0, pitch = 0.0, yaw = 0.0;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main(int _argc, char **_argv){
    std::cout << "Starting PID Control . . ." << '\n';

    //  Communication node with gazebo simulator
    gazebo::client::setup(_argc, _argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    //  Depth data subscriber
    gazebo::transport::SubscriberPtr depth_sub = node->Subscribe(
        "~/depth",
        &depth_unpack_callback);

    //  IMU sensor subscriber
    gazebo::transport::SubscriberPtr imu_sub = node->Subscribe(
        "~/pico/hull/imu_sensor/imu",
        &imu_unpack_callback);

    //  Thrust CMD publisher
    gazebo::transport::PublisherPtr thrust_cmd_pub = 
        node->Advertise<mini_auv_gazebo_msgs::msgs::ThrustCmd>("~/thrust_cmd");
    
    mini_auv_gazebo_msgs::msgs::ThrustCmd thrust_cmd;
    thrust_cmd.set_thruster1(0.0);
    thrust_cmd.set_thruster2(0.0);
    thrust_cmd.set_thruster3(0.0);
    thrust_cmd.set_thruster4(0.0);
    thrust_cmd.set_thruster5(0.0);
    thrust_cmd.set_thruster6(0.0);

    //  Maps the pid outputs from each pid controller
    //  to the individual thruster thrusts via a matrix mult.
    Matrix6d pid_thruster_mapper;
    pid_thruster_mapper << 1,  1,  0,  0,  0,  1,
                           0,  0,  1,  1,  0,  0,
                           1, -1,  0,  0,  0,  1,
                          -1, -1,  0,  0,  0,  1,
                           0,  0, -1,  1,  0,  0,
                          -1,  1,  0,  0,  0,  1;  

    //
    std::vector<ignition::math::PID> pid_controllers;

    //  Initialize PID controllers
    //  TODO: Add other PID controllers.
    ignition::math::PID roll_pid(0.0);
    roll_pid.SetPGain(0.0); roll_pid.SetIGain(0.0); roll_pid.SetDGain(0.0);
    roll_pid.SetCmdMax(0.1); roll_pid.SetCmdMin(-0.1);
    pid_controllers.push_back(roll_pid);

    ignition::math::PID pitch_pid(0.0);
    pitch_pid.SetPGain(0.0); pitch_pid.SetIGain(0.0); pitch_pid.SetDGain(0.0);
    pitch_pid.SetCmdMax(0.1); pitch_pid.SetCmdMin(-0.1);
    pid_controllers.push_back(pitch_pid);

    ignition::math::PID yaw_pid(0.0);
    yaw_pid.SetPGain(0.0); yaw_pid.SetIGain(0.0); yaw_pid.SetDGain(0.0);
    yaw_pid.SetCmdMax(0.1); yaw_pid.SetCmdMin(-0.1);
    pid_controllers.push_back(yaw_pid);

    ignition::math::PID x_pid(0.0);
    pid_controllers.push_back(x_pid);
    ignition::math::PID y_pid(0.0);
    pid_controllers.push_back(y_pid);

    ignition::math::PID z_pid(
       0.5, 0.05, 0.4, 0.1, -0.1, 0.3, -0.3, 0.0
    );
    z_pid.SetPGain(0.5); z_pid.SetIGain(0.05); z_pid.SetDGain(0.4);
    pid_controllers.push_back(z_pid);


    //  10 millisecond duration.
    std::chrono::duration<double> pid_loop_dur(0.010);
    
    while(true){
        
        //Compute error
        auto z_error = 10.0 - depth;
        auto roll_error = 0.0 - roll;   //This will need to account for angle wrapping
        auto pitch_error = 0.0 - pitch;
        auto yaw_error = 0.0 - yaw;
        auto x_error = 0.0;
        auto y_error = 0.0;

        //  It is critical to have the offset thrust on the z thrust since
        //  there is constant force acting on the vehicle in its equilibrium.
        //  This offset thrust can be calculated based on the dynamics of the vehicle.
        auto z_thrust = z_pid.Update(z_error, pid_loop_dur) - 2.54;
        auto roll_thrust = roll_pid.Update(roll_error, pid_loop_dur);
        auto pitch_thrust = pitch_pid.Update(pitch_error, pid_loop_dur);
        auto yaw_thrust = yaw_pid.Update(yaw_error, pid_loop_dur);
        auto x_thrust = 0.0;
        auto y_thrust = 0.0;

        Vector6d decoupled_thrusts;
        decoupled_thrusts << roll_thrust, pitch_thrust, yaw_thrust,
                                        x_thrust, y_thrust, z_thrust;

        //Map the decoupled thrusts (individual PID outputs) to the thrusters 
        //of the vehicle using the pid_thruster_mapper matrix.
        auto thrusts = pid_thruster_mapper * decoupled_thrusts;

        thrust_cmd.set_thruster1(thrusts[0]);
        thrust_cmd.set_thruster2(thrusts[1]);
        thrust_cmd.set_thruster3(thrusts[2]);
        thrust_cmd.set_thruster4(thrusts[3]);
        thrust_cmd.set_thruster5(thrusts[4]);
        thrust_cmd.set_thruster6(thrusts[5]);

        thrust_cmd_pub->Publish(thrust_cmd);

        gazebo::common::Time::MSleep(10);
    }

    return 0;
}

void depth_unpack_callback(DoublePtr &depth_msg){

    depth = depth_msg->data();
    std::cout << "Depth: " << depth << '\n';
}

void imu_unpack_callback(IMUPtr &imu_msg){
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

    /*
    std::cout << "Orientation --" << '\n';
    std::cout << "\tRoll:  " << roll << '\n';
    std::cout << "\tPitch: " << pitch << '\n';
    std::cout << "\tYaw:   " << yaw << '\n';
    */
}
