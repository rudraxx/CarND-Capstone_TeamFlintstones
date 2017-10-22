#!/bin/bash

# This is a file that will automatically build and start the package. 
# Was doing these 3 commands manually, and got tired of it.

echo "                   "
echo "                   "
echo "*******************"
echo "*******************"
echo "Starting ros build process... "
echo "*******************"
echo "*******************"
echo "                   "
echo "                   "

catkin_make

echo "*******************"
echo "*******************"
echo "Sourcing devel/setup.bash" 
echo "*******************"
echo "*******************"
echo "                   "
echo "                   "
source devel/setup.bash

echo "                   "
echo "                   "
echo "*******************"
echo "*******************"
echo "Launching the styx.launch file"
echo "*******************"
echo "*******************"
echo "                   "
echo "                   "
roslaunch launch/styx.launch


