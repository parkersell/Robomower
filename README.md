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
