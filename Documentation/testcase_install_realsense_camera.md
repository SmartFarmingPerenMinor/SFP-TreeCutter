#Testcase Install and connect Intel Realsense Camera

Prerequisites
•	Install ROS noetic
•	[optional] Install RVIZ
•	USB port
•	Intel Realsense Camera[tested with L515 and D415]

Go to the installation guide for the ROS intel realsense installation	https://github.com/IntelRealSense/realsense-ros#installation-instructions
Use method 1: The ROS distribution	sudo apt-get install ros-noetic-realsense2-camera
Connect the camera to the computer over USB	Test if the camera is visible with the “lsusb” terminal command
Run the camera rosnode 	roslaunch realsense2_camera rs_camera.launch
Show the running ros topics	rostopic list
The topics should include the following	/camera/depth/camera_info
/camera/color/camera_info
[optional] test using RVIZ by launching RVIZ	roslaunch rviz rviz
Create a new visual by clicking on add in RVIZ
Check for the right pointcloud2 topic and load it
If it gives an error make certain it has the right frame map	Check if you can see the camera feed in RVIZ

*if the topic is loaded the camera is installed correctly
*the camera terminal might give an error of not receiving data. This can be a temporary warning, you can still test if the camera feed is working.
