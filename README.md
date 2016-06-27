# usma_optitrack
Instructions for configuring a Natural Point Optitrack motion capture system in ROS. These instructions are based on this wiki: http://wiki.ros.org/mocap_optitrack. This ROS package's instructions are dated.

## Overview
The goal is to track multiple quadcopters using the OptiTrack motion capture system, then stream that data through a base station (your computer) and to the quadcopters with an onboard cpu running mavros, giving them a local pose estimate and allowing position setting, offboard control, etc. Instructions are compiled from a number of sources along with much trial and error.

## Motion Capture Setup
1. Turn on motion capture cameras and launch Motive.
2. Open most recent calibration file.
3. Create rigid bodies from selected markers (keep in mind that the orientation they are created in will be used as no rotation).
4. Open View -> Project
   - assign rigid bodies unique "User Data" fields. This is the ID that mocap\_optitrack will find them with.
   - offset pivot points of rigid bodies to reflect actual positions of the object they represent
5. Open View -> Data Streaming
   - Local Interface can be set to loopback or the computers own IP
   - Set Stream Rigid Bodies to true
   - Set Up Axis to Y Up (mocap\_optitrack remaps this in mocap\_optitrack/mocap_datapackets.cpp, although not entirely correctly)
   - Ensure Broadcast Frame Data is checked
   - Ensure Advanced Network Settings -> Type is set to Multicast, and set Multicast Interface to the base station's IP (hostname -I or ifconfig to find your IP).

## Base Station Setup
1. Read the mocap_optitrack [wiki](http://wiki.ros.org/mocap\_optitrack). The instructions are dated.
2. Clone this branch to your catkin workspace src folder: https://github.com/ros-drivers/mocap_optitrack/tree/new-and-old-support
   - if this branch is closed or gone, the master branch has probably updated and will be usable instead.
3. Update mocap\_optitrack/config/mocap.yaml to map all objects you would like to track (see vision\_pose.yaml for example). Do not map directly to namespace/mavros/vision_pose/pose, as coordinates are most likely still incorrectly rotated.
continue with other setup ... (cont after)

## Quad Setup
1. Install qgroundcontrol
2. Connect quad (and run calibration/setup if needed), then set parameters for using vision pose here: http://dev.px4.io/external-position.html
3. Adjust INAV weights and other parameters as needed.
4. Configure flight modes. Will need access to a manual mode or quad will not arm, offboard mode, and preferably position control mode.

## Offboard CPU Setup (ODROID or RPI assumed)
1. Get CPU set up with a wireless connection on same network as base station.
   - http://odroid.com/dokuwiki/doku.php
   - https://www.raspberrypi.org/help/noobs-setup/
2. Install mavros and mavros_extras: `sudo apt-get install ros-<yourdistro>-mavros` `sudo apt-get install mavros-<yourdistro>-mavros-extras`
3. Change default fcu\_url to match correct port and baud rate. If using multiple quads, each quad's copy of mavros MUST run under a unique namespace, or both quads will accept both sets of instructions. Copy and change mavros\_extras/launch/px4\_image.launch to launch everything under a unique namespace. For example:
   ```xml
   <launch>
     <arg name="ns" default="quad00" />
     <group ns="$(arg ns)">
       <arg name="fcu_url" default="serial:///dev/ttyUSB0:921600" />
       
       <!-- rest of cody body here -->
     
     </group>
   </launch>
   ```
   - USAGE: `roslaunch mavros_extras <launchfile.launch> ns:=<quadnamespace>`
4. Change mavros/launch/px4_pluginlists.yaml to blacklist any unused plugins (note: imu-pub is needed for local orientation).
5. Set up each offboard CPU to communicate with the base station
   - in each computer's .bashrc file (including base station), add:
     
     ```
     export ROS_MASTER_URI=http://<basestationIP>:11311
     export ROS_IP=<thisIP>
     ```
     
   - in each computer's /etc/hosts file (including base station), add mappings for all computer's IPs:
     
     ```
     192.168.200.*** basestation
     192.168.200.*** quad1
     192.168.200.*** etc.
     ```
     


## Base Station (cont.)
4. Correct coordinates with a node that subscribes to the topics published by mocap\_node and publishes to <namespace>/mavros/vision\_pose/pose. The correct rotation will have to be found experimentally; it depends on a variety of setup factors. See correct\_vis\_coords.cpp for an example on rotating this and calibrate.launch for an example on figuring out the correct rotation.

---

Ensure coordinates are correct and connection, power supply, etc. are good. When launching mavros on an offboard CPU, you should see `CON: Got HEARTBEAT, connected. FCU: PX4` if the mavlink connection is good and `FCU: [inav] VISION estimate valid` if the vision pose is published to the correct mavros topic. At this point test that the quad can hover in position control mode. Wear eye safety gear, as a dropped vision pose estimate in position control mode can cause the quad to fly uncontrollably.

At this point, you should also be able to feed the quads setpoint directions by publishing PoseStamped destinations to `<quadnamespace>/mavros/setpoint\_position/local`. These can be published to multiple quads independently. An example of a code structure that can publish scripts of various maneuvers to multiple quads in parallel is included in mocap\_optitrack/scripts. Example usage of this is included in multiquad\_script\_test. Any new QuadScripts must implement the virtual functions `init()`, `completed()`, and `publish_topic()`, as well as initialize any non-derived variables in the constructor.
