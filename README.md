# usma_optitrack
Instructions for configuring a Natural Point Optitrack motion capture system
Windows Setup
1. Install all dependencies (assuming ROS Indigo).
 * $ `sudo apt-get install ros-indigo-rosbridge-server`
 * $ `sudo apt-get install ros-indigo-mjpeg-server`
 * $ `sudo apt-get install ros-indigo-usb-cam`
 
2. Clone the usma_remote_interface ros package into your catkin workspace src folder.
  * $ `cd ~/catkin_ws/src`
  * $ `git clone https://github.com/westpoint-robotics/usma_remote_interface.git usma_remote_interface`
  
3. Run the setup script.
  * $ `cd ~/catkin_ws/src/usma_remote_interface/scripts`
  * $ `./usma_remote_interface_setup.sh`


Linux Setup


4. Edit the master.launch argument for the correct video input.
  * $ `cd ~/catkin_ws/src/usma_remote_interface/launch`
  * $ `ls /dev/video*` (to see available video devices)
  * $ `vim master.launch` (or your preferred editor)
  * Change the video device. Internal webcams will typically be video0, an external webcam could be video1.
  * `<arg name="video_device" default="/dev/video1" />`
