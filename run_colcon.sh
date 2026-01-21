#!/usr/bin/env bash
set -eo pipefail  # quitamos -u globalmente o lo gestionamos manualmente

cd $HOME/Final_Assigment/Final_Assigment-main/ros2_ws

set +u
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
set -u