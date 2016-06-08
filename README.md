# usma_optitrack
Instructions for configuring a Natural Point Optitrack motion capture system in ROS.  These instructions are based on this wiki: http://wiki.ros.org/mocap_optitrack.  This ROS package's instructions are dated.

## Windows Setup
1. Ensure the POE switch is turned on.
2. Launch Motive.
3. Open the calibration file (specify file here).
4. Create rigid bodies from selected markers.
5. Ensure broadcast rigid bodies box is checked.

## Linux Setup
1. Read the mocap_optitrack wiki: http://wiki.ros.org/mocap_optitrack
2. Clone the mocap_optitrack ROS package into your catkin workspace src folder.
 - $ `cd ~/catkin_ws/src`
 - $ `git clone https://github.com/ros-drivers/mocap_optitrack`
3. Change mocap.yaml to remap Rigid Body 1 marker to desired topic.
