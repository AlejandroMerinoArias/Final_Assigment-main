#!/usr/bin/env bash
set -eo pipefail  # quitamos -u globalmente o lo gestionamos manualmente

cd $HOME/Final_Assigment/Final_Assigment-test/ros2_ws

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

exec ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}