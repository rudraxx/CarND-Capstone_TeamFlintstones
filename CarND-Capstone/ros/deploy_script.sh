#!/bin/bash

echo "Starting ros build process... "

catkin_make

echo "Sourcing devvel/setup.bash" 
source devel/setup.bash

echo "Launching the styx.launch file"
roslaunch launch/styx.launch


