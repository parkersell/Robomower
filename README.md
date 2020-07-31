# Robomower
**Goal**: Low-cost, self-driven lawn mower with little to no setup.

We envision using Visual SLAM in combination with an IMU for motion blur.

Started with SVO and SVO2, but they are focused primarily on drone SLAM and SVO2 is for testing purposes only http://rpg.ifi.uzh.ch/svo2.html


Next we decided to run Orb Slam, since it has great documentation and usage, even though it might be a bit heavy for the Raspberry Pi.

Current Setup:
- Raspberry Pi 4
- Ubuntu Server 18.04 https://wiki.ubuntu.com/ARM/RaspberryPi
- Xubuntu-desktop (referenced in ^ link)
- ROS Melodic Desktop Full http://wiki.ros.org/melodic/Installation/Ubuntu
- orb_slam_2_ros https://github.com/appliedAI-Initiative/orb_slam_2_ros 
- libuvc_ros - USB Camera driver for ros http://wiki.ros.org/libuvc_camera (have not totally configured driver to run)

Tried:
- Raspicam (could not install ROS package properly on ubuntu) https://github.com/UbiquityRobotics/raspicam_node
- USB Cam ROS Node (publishes images on usb_cam/ rather than camera/ and could not get it to talk to Orb Slam 2) http://wiki.ros.org/usb_cam

We have also considered using these systems (found later in the project, and not attempted yet)
- https://github.com/rpng/open_vins
- https://github.com/xdspacelab/openvslam
- https://www.slamcore.com/products/software/ (not open source, but submitted early access form and were sent a questionnaire that we thought would be better to get further into development before submitting https://docs.google.com/forms/d/e/1FAIpQLSe-EvLTtv4Z5yiWVUd43EQ79LjG3PsoDuYCQ7mr8rROyb5-KQ/viewform)


**Tej and Parker's Update**

**Unix Password**: “ “(it’s one space)
- Password to get into the computer

**Important Terminal Commands**:

Realsense Camera: roslaunch realsense2_camera rs_rgbdwide.launch infra_fps:=15
- Launches Realsense camera node for d435 and 848x480 resolution, outputs infra1, infra2, and depth

ORBSLAM for Realsense: roslaunch orb_slam2_ros stereod435.launch
- Launches Stereo ORBSLAM for D435

Arduino Serial Node: source /opt/ros/melodic/setup.bash; rosrun rosserial_python serial_node.py /dev/ttyACM0  _baud:=115200 
- Enables communication between teensey and raspberry pi
- Compile on S0 port

Depth Image to Laserscan: roslaunch orb_slam2_ros depthscan.launch
- Converts D435 Laserscan to depth image 
- In order to convert to Float32: “cd Python” and run “python scan.py”
- This is so that it can be subscribed to in Arduino

Programming Arduino on PI: Click the Arduino IDE icon on the Desktop
- Ros.h errors when programming on Windows, thus we compiled the arduino code on the raspberry pi 

**Useful Stuff**:

Image Viewer: rosrun rqt_image_view rqt_image_view
- Visualizes any published ROS image topics

Rviz: rosrun rviz rviz
- Visualizes any image, laserscan, or pointcloud topic + more. 
- Does not run on VNC, only runs on physical PI

Rostopic list: rostopic list
- Lists all published ros topics

TightVNC: vncserver
- Tightvnc should launch on startup and automatically configure for the connected network. To relaunch run this command

Overclocked: the pi is overclocked at 2GHz
- This means it could overheat and that's why we display the degrees in Celsius in the top right corner. There is a decimal after the first two integers. For example, with 45277 as the number, it is really 45.277 degrees Celsius. 
- This can be changed in /boot/firmware/usercfg.txt which inherits the config.txt file and keeps it separate for ubuntu’s sake. 

**Stuff that you probably won’t need**:

Kinect Camera: roslaunch freenect_launch freenect.launch
- Launches kinect camera node
- Kinect requires power wires to connect to 12V power source

Kinect Orb Slam: roslaunch orb_slam2_ros kinect.launch
-Launches RGBD orb slam for kinect

Webcam launch + Orbslam: roslaunch orb_slam2_ros life.launch
- Launches Monocular camera and ORBSlam

**Arduino Code Explained**:



