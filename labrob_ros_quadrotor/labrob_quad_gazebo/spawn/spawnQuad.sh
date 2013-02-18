#!/bin/bash


. ${ROS_ROOT}/../../setup.bash
. ~/.bashrc
CONVERT_ROOT_START=`pwd`

# Get off three folders (main folder hypothesis)
cd ../../../
CONVERT_ROOT=`pwd`
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${CONVERT_ROOT}

# Presentation of file
echo "####################################################"
echo "#                    Spawning models               #"
echo "####################################################"

# Spawn robot models

# Quad
rosrun gazebo spawn_model -gazebo -file `rospack find labrob_quad_gazebo`/models/quadrotorWithCameraFuerte.sdf -model quad0 -x 0 -y 0 -z 1.0

echo "Spawn ended! Enjoy your gazebo world!"
