# usma_optitrack
## Overview
Instructions for configuring a Natural Point Optitrack motion capture system and VRPN client in order to stream pose data to ROS.
The goal is to track multiple rigid bodies using the OptiTrack motion capture system and stream pose data (position and orientation) to a base station computer.
Required:
1. Mocap computer - Windows desktop running Motive and VRPN server
2. Base station - Ubuntu laptop running ROS and VRPN client

## Motion capture computer setup
1. Turn on motion capture cameras and launch Motive.
2. Open most recent calibration file.
3. Create rigid bodies from selected markers (keep in mind that the orientation they are created in will be used as no rotation).
4. Open View -> Project
   - assign rigid bodies unique "User Data" fields. This is the ID that mocap\_optitrack will use to identify them.
   - offset pivot points of rigid bodies to reflect actual positions of the object they represent.
5. Open View -> Data Streaming
   - Local Interface can be set to loopback or the computers own IP.
   - Set Stream Rigid Bodies to True.
   - Set Up Axis to Y Up (mocap\_optitrack remaps this in src/mocap\_datapackets.cpp, although not correctly; further corrections will be needed).
   - Ensure Broadcast Frame Data is checked.
   - Ensure Advanced Network Settings -> Type is set to Multicast, and set Multicast Interface to the base station's IP (hostname -I or ifconfig to find your IP).

## Base station setup [[ref]](http://wiki.ros.org/vrpn_client_ros)
On a laptop with Ubuntu 16.04 and ROS Kinetic:
1. Clone this repositiory to your catkin workspace
``` 
cd ~/catkin_ws/src
git clone 
cd ..
catkin_make
```
3. Launch the basic.launch file: `roslaunch usma_optitrack basic.launch`
4. Echo ros topics to check for pose messages of the rigid body.
