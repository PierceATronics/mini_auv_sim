#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include  <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include "mini_auv_gazebo_msgs/Double.pb.h"

namespace gazebo{

    class DepthSensorPlugin : public ModelPlugin{

        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

            //  callback update called by simulator.
            void on_update();

            //  Returns emulated depth data relative to surface.
            double get_depth_rel_surface();

        private:

            event::ConnectionPtr update_connection;
            physics::ModelPtr model;
            sdf::ElementPtr sdf;
            transport::NodePtr node;

            transport::PublisherPtr depth_data_pub;

            //  position of specified surface in world. Default z = 0.0
            double surface_pos_z = 0.0;

            //  depth message.
            mini_auv_gazebo_msgs::msgs::Double depth_msg;
            
    };

}