# usma_optitrack
Instructions for configuring a Natural Point Optitrack motion capture system.

### Windows Setup
1. Launch Motive.
2. Open default calibration file.
3. Select rigid bodies.
4. Ensure broadcast rigid bodies bix is checked

### Linux Setup
1. Read the mocap_optitrack wiki: http://wiki.ros.org/mocap_optitrack
2. Clone the mocap_optitrack ROS package into your catkin workspace src folder.
  * $ `cd ~/catkin_ws/src`
  * $ `git clone https://github.com/ros-drivers/mocap_optitrack`
3. Change mocap.yaml to remap Rigid Body 1 marker to ...
4. 
