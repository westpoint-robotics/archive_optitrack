# usma_optitrack
Instructions for configuring a Natural Point Optitrack motion capture system in ROS. These instructions are based on this wiki: http://wiki.ros.org/mocap_optitrack. This ROS package's instructions are dated.

It may be possible to simply clone this repository, but configurations are not guaranteed to work with your setup, so you should still follow the guide below and configure everything to work with your setup.

View [these videos](https://drive.google.com/folderview?id=0Bwo0RaDbV3_oT0FuN3pRbEVLMms&usp=sharing) for examples done with this setup.

## Overview
The goal is to track multiple quadcopters using the OptiTrack motion capture system, then stream that data through a base station (your computer) and to the quadcopters with an onboard cpu running mavros, giving them a local pose estimate and allowing position setting, offboard control, etc. Instructions are compiled from a number of sources along with experimentation.

## Motion Capture Setup
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
