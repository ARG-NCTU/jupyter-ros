#/bin/bash

echo "Sourcing Kinetic base"
. /opt/ros/kinetic/setup.bash

echo "Moving to ros home directory"
cd /home/ros/

echo "Start rwt demos"
roslaunch rwt_demos.launch --screen
