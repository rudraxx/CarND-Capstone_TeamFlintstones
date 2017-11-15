# CarND-Capstone_TeamFlintstones


Shared project for completing the Udacity Self Driving Car Nano Degree Capstone Project

## Team Members

| Name                      | email                     |
|---------------------------|---------------------------|
| Abhishek Bhat             | abhishek.bhat04@gmail.com |
| Jos� Javier G�lvez Gamboa | jjaviergalvez@hotmail.com |
| Pawel Cisek               | pawel.a.cisek@gmail.com   |
| Madhu Kakarala            | madhutrix@gmail.com.      |
| Prashant Jain             | prashant.jain@gmail.com   |


## Video recording of Carla running on the simulator
https://drive.google.com/open?id=1MigL4t4nwG9c8Td4JFDQFhAsrzpWZ6co

## Additional notes about the roadblocks during project


### 1) Creating a shared folder on the vm

https://discussions.udacity.com/t/virtualbox-shared-folder-set-up/399166


### 2) Adding the permission for the student user.
https://stackoverflow.com/questions/26740113/virtualbox-shared-folder-permissions

### 3) Move file from vm desktop to vm folder
mv ~/Desktop/waypoint_updater.py ~/catkin_ws/src/capstone/CarND-Capstone/ros/src/waypoint_updater/waypoint_updater.py

### 4) Data logging in rospy

Use roscd log to get to the logfile. Then navigate to the node that you are interested in. Files in this log folder log all the rospy.loginfo for the given nodes.

### 5) Simulator dbw_enabled topic
Simulator has manual mode enabled from start. The dbw_enabled topic is published only when that checkbox is enabled or disabled. While starting the code, keeping the toggle_state to on.

### 6) Styx server has issues. The vehicle/throttle_cmd message wasnt being recieved by the server.
The resolution is provided in the discussion forum. (Adding codde to the styx server.py)
https://discussions.udacity.com/t/car-freezes-in-simulator-solved/363942/12
