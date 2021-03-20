#!/bin/bash
PROJ_DIR=$(pwd)
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PROJ_DIR}/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${PROJ_DIR}/worlds
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${PROJ_DIR}/media
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${PROJ_DIR}/build/mini_auv_gazebo_plugins
