#ifndef Subscriber_h
#define Subscriber_h
#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include "Point.h"
#include "Quat.h"

class Subscriber {
  public:
    Subscriber();
    ros::Subscriber<geometry_msgs::PoseStamped, Subscriber> pose_subscriber;

    void positionCallback(const geometry_msgs::PoseStamped&);
    Point getPosition();
    double getRoll();
    double getYaw();
    void initSLAM();
    void spinOnceS();

  private:
    inline double to_degrees(double radians) {
      return radians * (180.0 / M_PI);
    }

};

#endif
