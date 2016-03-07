# usma_optitrack
Instructions for configuring a Natural Point Optitrack motion capture system.

### Windows Setup
1. Launch Motive.
2. Open default calibration file.
3. Create rigid bodies (quad always first - Rigid_Body_1)
4. Open broadcasting side panel
5. Ensure "Broadcast rigid bodies" is set to True
6. Set mulitcast interface to latpop IP
7. Ensure broadcast frame data box is checked
8. On laptop: echo rostopic /mavros/vision_pose/pose #test position ouput of the quad

### Linux Setup
1. Read the mocap_optitrack wiki: http://wiki.ros.org/mocap_optitrack
2. Clone the mocap_optitrack ROS package into your catkin workspace src folder.
  * $ `cd ~/catkin_ws/src`
  * $ `git clone https://github.com/ros-drivers/mocap_optitrack`
3. Change mocap.yaml to remap Rigid Body 1 marker to ...
4. 
