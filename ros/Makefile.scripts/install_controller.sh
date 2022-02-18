#!/bin/bash

HERE=$(realpath $(dirname ${BASH_SOURCE:-$0}))
ROOT=$(realpath $HERE/../..)

CATKIN_WS=$ROOT/ros

set -e

source /opt/ros/kinetic/setup.bash

set -x

cd $CATKIN_WS

mkdir -p src/
catkin init

cd src
if [ ! -d franka_ros ]; then
  git clone https://github.com/frankaemika/franka_ros.git
fi
rosdep install --from-path . -r -y -i

catkin build reorient_ros
