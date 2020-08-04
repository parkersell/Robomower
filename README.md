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


# Tej and Parker's Update

**Unix Password**: “ “(it’s one space)
- Password to get into the computer

**Catkin Workspace**: catkin_ws
- There are other workspaces in home that we no longer use 
- Must use catkin_make -j2 to only use 2 cores when making, otherwise Pi will crash at 99% :)

**Pi will not Boot unless Program mode is enabled on Teensey or the serial wires are disconnected from the Pi** 
- This is because the Teensey sends junk because the serial console is not disabled

Important Links:
-
- https://github.com/appliedAI-Initiative/orb_slam_2_ros
- http://wiki.ros.org/navigation/Tutorials/RobotSetup
- http://cpp.sh/9ha24 Shell to run goToPosition command

Important Terminal Commands:
-

**Realsense Camera**: roslaunch realsense2_camera rs_rgbdwide.launch infra_fps:=15
- Launches Realsense camera node for d435 and 848x480 resolution, outputs infra1, infra2, and depth

**ORBSLAM for Realsense**: roslaunch orb_slam2_ros stereod435.launch
- Launches Stereo ORBSLAM for D435

**Arduino Serial Node**: source /opt/ros/melodic/setup.bash; rosrun rosserial_python serial_node.py /dev/ttyACM0  _baud:=115200 
- Enables communication between teensey and raspberry pi through the USB cable(also called ttyACM0)
- Compile on S0 port (white and grey wires running from pi to teensey)
- Must have both ports in use to send and recieve commands from the Pi

**Depth Image to Laserscan**: roslaunch orb_slam2_ros depthscan.launch
- Converts D435 Laserscan to depth image 
- In order to convert to Float32: “cd Python” and run “python scan.py”
- This is so that it can be subscribed to in Arduino

**Programming Arduino on PI**: Click the Arduino IDE icon on the Desktop
- Ros.h errors when programming on Windows, thus we compiled the arduino code on the raspberry pi 

Useful Stuff:
-

**Image Viewer**: rosrun rqt_image_view rqt_image_view
- Visualizes any published ROS image topics

**Rviz**: rosrun rviz rviz
- Visualizes any image, laserscan, or pointcloud topic + more. 
- Does not run on VNC, only runs on physical PI

**Rostopic list**: rostopic list
- Lists all published ros topics

**TightVNC**: vncserver
- Tightvnc should launch on startup and automatically configure for the connected network. To relaunch run this command

**Overclocked**: the pi is overclocked at 2GHz
- This means it could overheat and that's why we display the degrees in Celsius in the top right corner. There is a decimal after the first two integers. For example, with 45277 as the number, it is really 45.277 degrees Celsius. 
- This can be changed in /boot/firmware/usercfg.txt which inherits the config.txt file and keeps it separate for ubuntu’s sake. 

Stuff that you probably won’t need:
-

**Kinect Camera**: roslaunch freenect_launch freenect.launch
- Launches kinect camera node
- Kinect requires power wires to connect to 12V power source

**Kinect Orb Slam**: roslaunch orb_slam2_ros kinect.launch
-Launches RGBD orb slam for kinect

**Webcam launch + Orbslam**: roslaunch orb_slam2_ros life.launch
- Launches Monocular camera and ORBSlam

**RTabMap**:
- Has its own built in visual odometry
- Easily implemented with Navigation Stack but maybe less accurate than OrbSlam
- We didn't do that much testing with it
- http://wiki.ros.org/rtabmap_ros

Arduino Code:
-

**Hardware.cpp**
- Used to initialize Motors from Motor.h

**Movement.cpp**
- Documentation found in the Movement.h file

**Subscriber.cpp**
- Used to subscribe to rostopics on the Pi
- Have fun with the breakfast foods :)

**Point.h**
- Used to create a x and y point with Point p = Point(x,y);

**robomower.ino**
- All commands can be typed into the ttyS0 serial monitor 

PID and Motor was perfected when we got it


Future Improvements:
-
- Increase OrbSlam2 debug_image frame rate and accuracy
- Implement Dynamic Path Planning with things like Navigation Stack's costmap and movebase
- https://arxiv.org/pdf/1811.08414.pdf On page 5 there is a nice diagram of a solution to dynamic path planning
- http://wiki.ros.org/navigation/Tutorials/RobotSetup We followed this tutorial and put our code in robomower_2dnav but didn't get the chance to implement it with d435
- https://answers.ros.org/question/317419/realsense-d435-rtabmap-pixhawk-imu-robot_localization/?answer=318026#post-id-318026 Consider fusing IMU data and Wheel Encoder data with SLAM data to create more accurate and robust localization 
- http://wiki.ros.org/rtabmap_ros/Tutorials/RemoteMapping Consider sending sensor data over WiFi to another laptop or computer with ROS that then computes, maps, and sends velocity commands back to the Pi. This would allow for more advanced computations
- Use a usb processor for the pi to enhance computational abilities https://www.amazon.com/Google-Coral-Accelerator-coprocessor-Raspberry/dp/B07R53D12W or https://store.intelrealsense.com/buy-intel-neural-compute-stick-2.html?cid=sem&source=sa360&campid=2019_q3_egi_us_ntgrs_nach_revs_text-link_brand_bmm_desk_realsense-shopping-ad_o-1lngr_google&ad_group=RealSense+Shopping+Ads&intel_term=PRODUCT_GROUP&sa360id=92700050119513690&gclid=CjwKCAjwjqT5BRAPEiwAJlBuBVPIIMYaDe-IktebsqEzQGetq9hvNiPv6N6X9qo8CBZLc4SeoZ6MlxoCwLYQAvD_BwE&gclsrc=aw.ds





