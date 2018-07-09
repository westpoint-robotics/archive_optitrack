Instructions for configuring a Natural Point Optitrack motion capture system in ROS. These instructions are based on this wiki: http://wiki.ros.org/mocap_optitrack. 
This ROS package's instructions are dated.
View [these videos](https://drive.google.com/folderview?id=0Bwo0RaDbV3_oT0FuN3pRbEVLMms&usp=sharing) for examples done with this setup.
## Base Station Setup
1. Read the mocap\_optitrack [wiki](http://wiki.ros.org/mocap_optitrack) for basic overview. Note that instructions are dated.
2. Clone this branch to your catkin workspace src folder: https://github.com/ros-drivers/mocap_optitrack/tree/new-and-old-support
   - This branch addresses problems with the new version of Motive. If this branch is closed or behind, it may have been pushed onto the master branch, in which case you should just clone that.
3. Update mocap\_optitrack/config/mocap.yaml to map all objects you would like to track (see config/vision\_pose.yaml for example). Do not map directly to <namespace>/mavros/vision_pose/pose, as coordinates are most likely still incorrectly rotated.
continue with other setup (cont after).

## Quad Setup
1. Install qgroundcontrol
2. Connect quad (and run calibration/setup if needed), then set parameters for using vision pose [here](http://dev.px4.io/external-position.html).
3. If flying multiple quads, you will need to assign them unique MAVLINK_SYSID parameters.
4. Adjust INAV weights and other parameters as needed.
5. Configure flight modes. Will need access to a manual mode or quad will not arm, offboard mode, and preferably position control mode.
6. [Configure](http://www.spektrumrc.com/prodinfo/files/dx7_manual.pdf) a transmitter to control arming and mode settings.

## Offboard CPU Setup (ODROID or RPI assumed)
1. Get CPU set up with a wireless connection on same network as base station.
   - http://odroid.com/dokuwiki/doku.php
   - https://www.raspberrypi.org/help/noobs-setup/
2. Install mavros and mavros\_extras on the offboard cpus:

   ```
   sudo apt-get install ros-<yourdistro>-mavros
   sudo apt-get install mavros-<yourdistro>-mavros-extras
   ```
3. Change default fcu\_url to match correct port and baud rate. If using multiple quads, each quad's copy of mavros MUST run under a unique namespace  with unique target system IDs (matches MAVLINK\_SYSID set in qgroundcontrol). Copy and change mavros\_extras/launch/px4\_image.launch to launch everything under a unique namespace. For example:
   ```xml
   <launch>
     <arg name="ns" default="quad00" />
     <group ns="$(arg ns)">
       <arg name="fcu_url" default="serial:///dev/ttyUSB0:921600" />
       <arg name="gcs_url" default="udp://@" />
       <arg name="tgt_system" default="2" />

       <!-- rest of launch file -->

     </group>
   </launch>
   ```
   - USAGE: `roslaunch mavros_extras <launchfile> ns:=<quadnamespace>`
4. Change mavros/launch/px4\_pluginlists.yaml to blacklist any unused plugins (note: imu-pub is needed for local orientation).
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
4. Correct coordinates with a node that subscribes to the topics published by mocap\_node and publishes to `QUAD_NS/mavros/vision\_pose/pose`. Z axis is most likely correct, but the xy plane may be rotated from the correct orientation; it depends on a variety of setup factors. See src/correct\_vis\_coords.cpp for an example on rotating this and launch/calibrate.launch for an example on figuring out the correct rotation.

---

Ensure coordinates are correct and connection, power supply, etc. are good. SSH to the quad's offboard computer and launch mavros_extras, and run mocap_node and your coordinate correction node on the base computer. An example of this compiled in a launch file is found in launch/vision_pose.launch. You should see `CON: Got HEARTBEAT, connected. FCU: PX4` if the mavlink connection is good and `FCU: [inav] VISION estimate valid` if the vision pose is published to the correct mavros topic. At this point test that the quad can hover in position control mode. Wear eye safety gear, as a dropped network connection or incorrectly rotated coordinates can cause the quad to fly uncontrollably.

At this point, you should also be able to feed the quads setpoint directions by publishing PoseStamped destinations to `QUAD_NS/mavros/setpoint_position/local`. These can be published to multiple quads independently.

---

## Ad Hoc Network Setup
The ROS network can also be set up using an ad hoc network to avoid latency or other interference from other traffic on the network.

1. Configure an ad hoc network as shown here: https://wiki.debian.org/WiFi/AdHoc. You may have to use a serial connection.
2. Add permanent arp entries for each quad.
   - add IP to mac address mappings for each other device on the network in /etc/ethers (may have to create this file).
     You can find the mac addresses with `ifconfig <devicename>`.

     ```
     192.168.1.2     qu:ad:1m:ac:ad:dr
     192.168.1.3     qu:ad:2m:ac:ad:dr
     ```

   - Run `arp -f` as root to load these addresses now.
   - Configure these to load on boot for each odroid: add `arp -f` just before the `exit 0` in /etc/rc.local
   - Try rebooting each offboard computer, you should be able to ping/ssh to each quad at this point.
   - If network does not work, check device settings with `iwconfig <devicename>` and check that the mac addresses are correct with `arp`.

   ---

## Script Examples
An example of code used to write and run scripts for multiple quads is included in include/mocap_optitrack under Quad.h and QuadScripts.h (with corresponding source code in src/Quad.cpp and src/QuadScripts.cpp, with a runfile in src/multiquad_script_example.cpp).

Each Quad object corresponds to an individual quadcopter, identified by namespace. It holds data about the quad in its data struct. View Quad.h for specific method information and src/Quad.cpp for implementation. Each Quad object holds a script of (pointers to) QuadScript objects, which it runs in sequence. It also has a list of other quads that it knows about (can access position, velocity, etc).

Each QuadScript object gives instructions to the Quad it belongs to when it is run. Each "type" of script (e.g. takeoff, set position, follow other quad, catch a ball, etc.) is implemented as a derived class of QuadScript. Each QuadScript has access to the data struct of its parent Quad, and uses this to calculate instructions, then publishes an instruction message under QUAD_NS/mavros/setpoint_position/local.
  If writing new scripts, look at SetPose for a simple example on infrastructure. Each new script must implement the pure virtual functions init(), completed(), and publish_topic(). Descriptions of these functions are in QuadScripts.h

Quads can optionally be grouped into Formation objects, allowing easier control for formation movement and control. They contain a vector of Quad objects, and can add scripts to groups of Quads at once (although the quads still run them independently). It also makes lets every quad in the formation know where every other quad in the formation is, and implements boundary checking between the quads, disarming them if they come too close to each other.

This is just an example of a structure that uses setpoint positions; your own can be written and can also use setpoint velocity, acceleration, attitude, etc.
