#include "DepthSensorPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(DepthSensorPlugin);

void DepthSensorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    this->model = _parent;
    this->sdf = _sdf;

    //  Connect update callback to Gazebo world update event.
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DepthSensorPlugin::on_update, this)
    );

    //  Initialize node transport and publisher.
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();

    this->depth_data_pub = this->node->Advertise<mini_auv_gazebo_msgs::msgs::Double>("~/depth");
}

void DepthSensorPlugin::on_update(){

    double depth = get_depth_rel_surface();

    //  TODO: Add Gaussian Noise

    depth_msg.set_data(depth);

    this->depth_data_pub->Publish(depth_msg);
    //  publish the depth data. Hmmm, maybe seperate thread to control udpate rate.
}

double DepthSensorPlugin::get_depth_rel_surface(){
    double depth;

    ignition::math::Pose3d world_pos = this->model->WorldPose();

    depth = this->surface_pos_z - world_pos.Z();
    return(depth);
}
