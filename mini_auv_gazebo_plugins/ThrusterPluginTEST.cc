#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/common.hh>
#include "mini_auv_gazebo_msgs/ThrustCmd.pb.h"
#include "mini_auv_gazebo_msgs/Double.pb.h"


int main(int _argc, char **_argv){

  //Connect this node with the gazebo system
  gazebo::client::setup(_argc, _argv);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::PublisherPtr thrustCmdPub;

  thrustCmdPub = node->Advertise<mini_auv_gazebo_msgs::msgs::ThrustCmd>("~/thrust_cmd");
  mini_auv_gazebo_msgs::msgs::ThrustCmd thrustCmd;

  while(true){

    for(int i = 0; i<100; i++){
      gazebo::common::Time::MSleep(10);
      double thrust = -0.05 * (double)i;
      thrustCmd.set_thruster1(0);
      thrustCmd.set_thruster2(10);
      thrustCmd.set_thruster3(0);
      thrustCmd.set_thruster4(0);
      thrustCmd.set_thruster5(-10);
      thrustCmd.set_thruster6(0);
    
      thrustCmdPub->WaitForConnection();
      thrustCmdPub->Publish(thrustCmd);
    }
    gazebo::common::Time::MSleep(500);
    for(int i = 99; i>=50; i--){
      gazebo::common::Time::MSleep(10);
      double thrust = -0.05 * (double)i;
      thrustCmd.set_thruster1(0);
      thrustCmd.set_thruster2(10);
      thrustCmd.set_thruster3(0);
      thrustCmd.set_thruster4(0);
      thrustCmd.set_thruster5(-10);
      thrustCmd.set_thruster6(0);
    
     thrustCmdPub->WaitForConnection();
      thrustCmdPub->Publish(thrustCmd);
    }

    
  }
  return 0;
}
