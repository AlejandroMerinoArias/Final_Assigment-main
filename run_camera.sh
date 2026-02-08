#!/usr/bin/env bash
set -eo pipefail  # quitamos -u globalmente o lo gestionamos manualmente

cd $HOME/Final_Assigment/Final_Assigment-test/ros2_ws

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

export GALLIUM_DRIVER=d3d12
exec ros2 launch camera_recorder_pkg camera_recorder.launch.py "$@"