#ifndef Subscriber_h
#define Subscriber_h
#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "Point.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class Subscriber {
  public:
    Subscriber();

    //Subscribers
    ros::Subscriber<geometry_msgs::PoseStamped, Subscriber> pose_subscriber;
    ros::Subscriber<std_msgs::Float32, Subscriber> laser_subscriber;
    void laserCallback(const std_msgs::Float32&);
    void positionCallback(const geometry_msgs::PoseStamped&);

    //gets
    Point getPosition();
    double getLaser();
    double getRoll();
    double getYaw();

    //SLAM
    void initSLAM();
    void spinOnceS();
    bool tracking();

   //IMU
   Adafruit_BNO055 bno;
    void setupIMU();
    double getIMU();
    void resetIMU();
    void sendIMU();
    double constrainAngle(double);

  private:
    inline double to_degrees(double radians) {
      return radians * (180.0 / M_PI);
    }

};

#endif
