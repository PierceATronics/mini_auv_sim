#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <assert.h>
#include "mini_auv_gazebo_msgs/Double.pb.h"
#include "mini_auv_gazebo_msgs/ThrustCmd.pb.h"

namespace gazebo
{

  typedef const boost::shared_ptr<const msgs::Int> IntPtr;
  typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::Double> DoublePtrr;
  typedef const boost::shared_ptr<const mini_auv_gazebo_msgs::msgs::ThrustCmd> ThrustCmdPtr;

  class ThrusterPlugin : public ModelPlugin
  {
    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
      {
        //store the point to the model
        this->model = _parent;
        this->modelSdf = _sdf;

        //Connect listener for update event from simulator
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ThrusterPlugin::OnUpdate, this));

        std::string robotNameSpace = this->model->GetName();

        //Node for receiving thruster commands
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        this->thrusterCmdSub = this->node->Subscribe("~/thrust_cmd",
          &ThrusterPlugin::thrustCmdCallback, this);

        thrusterLinks.push_back(model->GetLink("thruster_1"));
        thrusterLinks.push_back(model->GetLink("thruster_2"));
        thrusterLinks.push_back(model->GetLink("thruster_3"));
        thrusterLinks.push_back(model->GetLink("thruster_4"));
        thrusterLinks.push_back(model->GetLink("thruster_5"));
        thrusterLinks.push_back(model->GetLink("thruster_6"));
        //Make smart method for selecting thrusters.
        //assert(this->modelSdf->HasElement("thruster"));
      }

      //On each world update, update the force applied at each thruster.
      void OnUpdate()
      {

          
          thrusterLinks[0]->AddRelativeForce(ignition::math::Vector3d(0, 0, thrusts[0]));
          thrusterLinks[1]->AddRelativeForce(ignition::math::Vector3d(thrusts[1], 0, 0));
          thrusterLinks[2]->AddRelativeForce(ignition::math::Vector3d(0, 0, thrusts[2]));
          thrusterLinks[3]->AddRelativeForce(ignition::math::Vector3d(0, 0, thrusts[3]));
          thrusterLinks[4]->AddRelativeForce(ignition::math::Vector3d(thrusts[4],0, 0));
          thrusterLinks[5]->AddRelativeForce(ignition::math::Vector3d(0, 0, thrusts[5]));


      }

      void thrustCmdCallback(ThrustCmdPtr &thrustCmdMessage)
      {
        //std::cout << thrustCmdMessage->data() << std::endl;
        //std::cout << thrustCmdMessage->thruster1() << std::endl;

        //Extract the thruster values.
        thrusts[0] = thrustCmdMessage->thruster1();
        thrusts[1] = thrustCmdMessage->thruster2();
        thrusts[2] = thrustCmdMessage->thruster3();
        thrusts[3] = thrustCmdMessage->thruster4();
        thrusts[4] = thrustCmdMessage->thruster5();
        thrusts[5] = thrustCmdMessage->thruster6();

        /*
        std::cout << "\nThrusts:" << std::endl;
        
        for(int i=0; i < 6; i++){
          std::cout << "/tThruster " << i << ": " << thrusts[i] << std::endl;
        }
        */
      }

    private:
      physics::ModelPtr model;
      sdf::ElementPtr modelSdf;
      event::ConnectionPtr updateConnection;
      transport::NodePtr node;
      transport::SubscriberPtr thrusterCmdSub;


      std::vector<physics::LinkPtr> thrusterLinks;
      double thrusts[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)
}
