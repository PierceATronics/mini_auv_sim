#!/bin/bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/pierce/Projects/mini_auv_sim/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/home/pierce/Projects/mini_auv_sim/worlds
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/home/pierce/Projects/mini_auv_sim/media
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/pierce/Projects/mini_auv_sim/build/mini_auv_gazebo_plugins
